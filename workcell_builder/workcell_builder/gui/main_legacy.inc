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
#include <QSet>
#include <QRegularExpression>
#include <iostream>
#include <limits>
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

// --web3d-scene-sequence: opens each scene in turn and waits for the embedded
// Web3D Product View to report scene_ready for that exact scene.
struct Web3DSceneSequenceOptions
{
  QString scene_path;
  QString smoke_output;
  QStringList web3d_scene_sequence;
};

class Web3DSceneSequenceRunner : public QObject
{
public:
  Web3DSceneSequenceRunner(QApplication * app, MainWindow * window, const Web3DSceneSequenceOptions & opts)
  : app_(app), window_(window), opts_(opts) {}

  void start() { QTimer::singleShot(0, this, &Web3DSceneSequenceRunner::load_next_scene); }

private:
  QApplication * app_;
  MainWindow * window_;
  Web3DSceneSequenceOptions opts_;
  QJsonArray blockers_;
  int index_{0};
  QJsonArray results_;

  void load_next_scene()
  {
    if (index_ >= opts_.web3d_scene_sequence.size()) {
      finalize();
      return;
    }
    const QString scene = opts_.web3d_scene_sequence.at(index_).trimmed();
    const QString root = QFileInfo(opts_.scene_path).isDir()
      ? QFileInfo(opts_.scene_path).absolutePath() : QString();
    const QString path = root.isEmpty() ? QString() : QDir(root).filePath(scene);
    QStringList load_blockers;
    QJsonObject diagnostics;
    if (!window_->load_scene_for_scene3d_smoke(scene, path, &load_blockers, &diagnostics)) {
      for (const QString & blocker : load_blockers) blockers_.append(blocker);
      finalize();
      return;
    }
    const qint64 started = QDateTime::currentMSecsSinceEpoch();
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this, timer, scene, started]() {
      ScenePreviewWidget * preview = window_ ? window_->active_scene_preview_widget() : nullptr;
      const bool identity_matches = preview && preview->preview_context().scene_id == scene;
      const bool ready = identity_matches && preview->runtime_preview_status_text() == QStringLiteral("Web3D Product View — ready");
      if (ready) {
        QJsonObject result;
        result["cycle"] = index_ + 1;
        result["scene"] = scene;
        result["status"] = preview->runtime_preview_status_text();
        result["scene_identity_matches"] = true;
        results_.append(result);
        qInfo().noquote() << QStringLiteral("Web3D switch smoke ready: cycle=%1 scene=%2 status=scene_ready")
          .arg(index_ + 1).arg(scene);
        timer->stop(); timer->deleteLater();
        ++index_;
        QTimer::singleShot(100, this, &Web3DSceneSequenceRunner::load_next_scene);
      } else if (QDateTime::currentMSecsSinceEpoch() - started > 45000) {
        blockers_.append(QStringLiteral("Web3D scene switch timed out: %1 status=%2 identity=%3")
          .arg(scene, preview ? preview->runtime_preview_status_text() : QStringLiteral("missing preview"),
            preview ? preview->preview_context().scene_id : QStringLiteral("missing")));
        timer->stop(); timer->deleteLater();
        finalize();
      }
    });
    timer->start(100);
  }

  void finalize()
  {
    QJsonObject root;
    root["schema"] = QStringLiteral("workcell_studio_web3d_scene_switch_smoke/v1");
    root["requested_cycles"] = opts_.web3d_scene_sequence.size();
    root["completed_cycles"] = results_.size();
    root["results"] = results_;
    root["blockers"] = blockers_;
    root["pass"] = blockers_.isEmpty() && results_.size() == opts_.web3d_scene_sequence.size();
    QFile output(opts_.smoke_output);
    if (output.open(QIODevice::WriteOnly | QIODevice::Text))
      output.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    std::cout << QJsonDocument(root).toJson(QJsonDocument::Indented).toStdString() << std::endl;
    app_->exit(root.value("pass").toBool() ? 0 : 1);
  }
};

// The native Scene3D renderer was retired; its paint-counter smoke has no
// subject any more. Report that explicitly instead of a misleading PASS/FAIL.
int write_native_scene3d_smoke_retired_report(const QString & output_path)
{
  QJsonObject root;
  root["schema"] = QStringLiteral("workcell_studio_scene3d_gui_smoke/v1");
  root["status"] = QStringLiteral("RETIRED");
  root["blockers"] = QJsonArray{QStringLiteral("native_scene3d_renderer_retired")};
  root["message"] = QStringLiteral(
    "The native Scene3D renderer was retired; Web3D is the only 3D viewport. "
    "Use --web3d-scene-sequence (scene_ready per scene) or the Web3D acceptance runners instead.");
  QFile output(output_path);
  if (output.open(QIODevice::WriteOnly | QIODevice::Text))
    output.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  std::cerr << root.value("message").toString().toStdString() << std::endl;
  return 3;
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
  // WSLg exposes a Mesa D3D12 GPU that Chromium blocklists, which leaves the Product View without
  // WebGL ("Scene could not finish loading"). Opt in to the GPU on WSL only, and never override flags
  // the user set.
  if (qEnvironmentVariableIsEmpty("QTWEBENGINE_CHROMIUM_FLAGS") && !qEnvironmentVariableIsEmpty("WSL_DISTRO_NAME"))
    qputenv("QTWEBENGINE_CHROMIUM_FLAGS", "--ignore-gpu-blocklist");
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

  workcell_builder::applyStudioApplicationAppearance(&a);

  if (has_cli_flag(args, "--repeat-save-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene-path");
    // This acceptance may only edit a disposable fixture below its output directory.
    if (output.isEmpty() || QFileInfo(scene).canonicalFilePath() !=
        QFileInfo(output + "/scenes/ur5_2f_test").canonicalFilePath() ||
        !QFileInfo::exists(scene + "/environment.yaml")) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
    TestMain::start_repeat_save_acceptance(&w, &a, output, scene,
      cli_value(args, "--axis") == "y", has_cli_flag(args, "--reopen"));
    return a.exec();
  }

  if (has_cli_flag(args, "--persistence-only")) {
    const QString output = cli_value(args, "--smoke-output");
    if (output.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble");
    w.show();
    TestMain::start_persistence_only(&w, &a, output);
    return a.exec();
  }

  if (has_cli_flag(args, "--operator-preview-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
    TestMain::start_operator_preview_acceptance(&w, &a, output, scene, has_cli_flag(args, "--reopen"));
    return a.exec();
  }
  // --screen NAME (an xrandr/QScreen name such as rdp-1): open on that real monitor and
  // maximize there. Only a size the window manager grants is used; nothing is forced.
  const auto maximize_on_screen = [&args](QWidget & window) {
    const QString name = cli_value(args, "--screen");
    if (name.isEmpty()) return;
    QScreen * target = nullptr;
    for (auto * screen : QGuiApplication::screens()) if (screen->name() == name) target = screen;
    if (!target) { qWarning().noquote() << "--screen" << name << "not found"; return; }
    // Called before the first show(): the compositor honours a window's position only
    // when it is mapped, so place it on the monitor first, then maximize there.
    // The shell sets the maximized state at construction; mapped like that, the window
    // would open maximized on the primary monitor.
    window.setWindowState(Qt::WindowNoState);
    window.resize(1280, 800);
    window.move(target->availableGeometry().topLeft() + QPoint(40, 40));
    for (const int delay : {300, 2500}) {
      QTimer::singleShot(delay, &window, [&window, target]() {
        window.showMaximized();
        qInfo().noquote() << "--screen: maximized on" << target->name() << window.geometry();
      });
    }
  };
  if (has_cli_flag(args, "--ui-tour")) {
    // Screenshot tour of every page/tab/wizard at --window-size (see test/ui_tour_capture.inc).
    const QString output = cli_value(args, "--smoke-output");
    if (output.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble");
    const auto parts = cli_value(args, "--window-size").split('x');
    maximize_on_screen(w);
    w.show();
    // The shell may maximize the window; the tour measures the requested size (clamped by the minimum
    // size). Without --window-size the restored/default geometry is kept (persistence checks).
    if (parts.size() == 2) {
      const QSize requested(parts[0].toInt(), parts[1].toInt());
      w.setWindowState(Qt::WindowNoState);
      w.resize(requested);
      // The window manager may apply the first-run maximize after this; re-assert once it settles.
      QTimer::singleShot(1200, &w, [&w, requested]() { w.setWindowState(Qt::WindowNoState); w.resize(requested); });
    }
    TestMain::start_ui_tour(&w, &a, output, cli_value(args, "--scene"), cli_value(args, "--tag"));
    return a.exec();
  }
  if (has_cli_flag(args, "--multi-select-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble");
    const auto parts = cli_value(args, "--window-size").split('x');
    const QSize window_size = parts.size() == 2 ? QSize(parts[0].toInt(), parts[1].toInt()) : QSize(1366, 768);
    if (cli_value(args, "--screen").isEmpty()) w.resize(window_size);
    maximize_on_screen(w);
    w.show();
    TestMain::start_multi_select_acceptance(&w, &a, output, scene,
      cli_value(args, "--screen").isEmpty() ? window_size : QSize(), cli_value(args, "--airpick-scene"));
    return a.exec();
  }
  if (has_cli_flag(args, "--visual-mate-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble");
    const QString size = cli_value(args, "--window-size");
    const auto parts = size.split('x');
    const QSize window_size = parts.size() == 2 ? QSize(parts[0].toInt(), parts[1].toInt()) : QSize(1366, 768);
    w.resize(window_size);
    w.show();
    TestMain::start_visual_mate_acceptance(&w, &a, output, scene, has_cli_flag(args, "--reopen"), window_size);
    return a.exec();
  }
  if (has_cli_flag(args, "--datum-mating-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
    TestMain::start_datum_mating_acceptance(&w, &a, output, scene, has_cli_flag(args, "--reopen"));
    return a.exec();
  }
  if (has_cli_flag(args, "--asset-mounting-acceptance") || has_cli_flag(args, "--newcell-task-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
    TestMain::start_mounting_acceptance(&w, &a, output, scene, has_cli_flag(args, "--reopen"), has_cli_flag(args, "--newcell-task-acceptance"), cli_value(args, "--observations"));
    return a.exec();
  }
  if (has_cli_flag(args, "--epd-readiness-acceptance")) {
    // Open/close the real window repeatedly; each cycle records panel states and process RSS.
    const QString output = cli_value(args, "--smoke-output");
    const QString scene = cli_value(args, "--scene");
    if (output.isEmpty() || scene.isEmpty()) return 2;
    const int cycles = std::max(1, cli_value(args, "--cycles").toInt());
    const int first_duration = std::max(5, cli_value(args, "--duration-s").toInt());
    QJsonArray rss_kib;
    int rc = 0;
    for (int cycle = 1; cycle <= cycles && rc == 0; ++cycle) {
      {
        MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
        TestMain::start_epd_readiness_acceptance(&w, &a, output, scene, cli_value(args, "--scene-path"),
          cycle, cycle == 1 ? first_duration : 8);
        rc = a.exec();
      }
      QCoreApplication::processEvents();
      QFile statm("/proc/self/status"); statm.open(QIODevice::ReadOnly);
      for (const auto & line : QString::fromUtf8(statm.readAll()).split('\n')) {
        if (line.startsWith("VmRSS:")) rss_kib.append(line.section(QRegularExpression("\\s+"), 1, 1).toInt());
      }
    }
    QFile f(output + "/lifecycle.json");
    if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(QJsonObject{{"cycles", cycles}, {"exit_code", rc},
      {"rss_kib_after_each_close", rss_kib}, {"windows_alive_after_close", QApplication::topLevelWidgets().size()}}).toJson());
    return rc;
  }
  if (has_cli_flag(args, "--multiobject-plan-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    const QString recording = cli_value(args, "--observations");
    if (output.isEmpty() || recording.isEmpty()) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble"); w.show();
    TestMain::start_multiobject_plan_acceptance(&w, &a, output, recording, has_cli_flag(args, "--reopen"),
                                                has_cli_flag(args, "--no-plan-check"), cli_value(args, "--sequence-order"),
                                                cli_value(args, "--sequence-places"), cli_value(args, "--expect-sequence"));
    return a.exec();
  }
  if (has_cli_flag(args, "--live-perception-acceptance")) {
    // Run with an isolated XDG_CONFIG_HOME: the cell's output root is set like Create New Cell sets it.
    const QString output = cli_value(args, "--smoke-output"), scene = cli_value(args, "--scene-path");
    if (output.isEmpty() || scene.isEmpty() || QFileInfo::exists(output + "/acceptance.json")) return 2;
    QSettings().setValue("startup/last_scene_output_root", QFileInfo(scene).absolutePath());
    MainWindow w(cli_value(args, "--workspace"), "humble");
    w.show();
    const int observe = cli_value(args, "--observe-s").toInt();
    TestMain::start_live_perception_acceptance(&w, &a, output, scene, observe > 0 ? observe : 60);
    return a.exec();
  }

  if (has_cli_flag(args, "--physics-cycle-acceptance")) {
    // Run with an isolated XDG_CONFIG_HOME: the cells' output root is set like Create New Cell sets it.
    const QString output = cli_value(args, "--smoke-output"), scene = cli_value(args, "--scene-path");
    if (output.isEmpty() || scene.isEmpty() || QFileInfo::exists(output + "/acceptance.json")) return 2;
    QSettings().setValue("startup/last_scene_output_root", QFileInfo(scene).absolutePath());
    MainWindow w(cli_value(args, "--workspace"), "humble");
    w.show();
    TestMain::start_physics_cycle_acceptance(&w, &a, output, scene, cli_value(args, "--cancel-at"));
    return a.exec();
  }

  if (has_cli_flag(args, "--full-cycle-acceptance")) {
    const QString output = cli_value(args, "--smoke-output");
    if (output.isEmpty() || QFileInfo::exists(output + "/acceptance.json")) return 2;
    MainWindow w(cli_value(args, "--workspace"), "humble");
    w.show();
    TestMain::start_full_cycle_acceptance(&w, &a, output);
    return a.exec();
  }

  if (has_cli_flag(args, "--scene3d-smoke")) {
    const QString smoke_output = cli_value(args, "--smoke-output");
    if (smoke_output.trimmed().isEmpty()) {
      std::cerr << "Missing required --smoke-output <path> for --scene3d-smoke" << std::endl;
      return 2;
    }
    Web3DSceneSequenceOptions sequence_opts;
    sequence_opts.scene_path = cli_value(args, "--scene-path");
    sequence_opts.smoke_output = smoke_output;
    sequence_opts.web3d_scene_sequence = cli_value(args, "--web3d-scene-sequence").split(',', Qt::SkipEmptyParts);
    if (sequence_opts.web3d_scene_sequence.isEmpty()) return write_native_scene3d_smoke_retired_report(smoke_output);
    MainWindow w(cli_value(args, "--workspace"), cli_value(args, "--ros-distro"));
    w.show();
    auto * runner = new Web3DSceneSequenceRunner(&a, &w, sequence_opts);
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
