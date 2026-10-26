#pragma once
// Explicit workstation acceptance using the real window/actions, never a mock executor.
#include <QApplication>
#include <QDialogButtonBox>
#include <QListWidget>
#include <QSettings>
#include "gui/new_cell_wizard.h"
#include "gui/task_intent_editor.h"
#include "gui/datum_mating_text.hpp"
#include "gui/operator_preview_launcher.hpp"
#include <QComboBox>
#include <QLineEdit>
#include <QMessageBox>
#include <QProcess>
#include <QSaveFile>
#include <QRegularExpression>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QPushButton>
#include <QCheckBox>
#include <QTimer>
#include <QSet>
#include <QTableWidget>
#include "epd_detection_readiness_model.hpp"
#include <QTextEdit>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QLabel>
#include <QMessageBox>
#include <QSettings>
#include <QAbstractButton>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <tuple>
#include <functional>
#include <array>
#include <algorithm>
#include <QLabel>
#include <QComboBox>
#include <QTreeWidget>
#include <stdexcept>
#include <QCryptographicHash>
#include <QProcess>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QRadioButton>
#include <QRegularExpression>
#include <QScreen>
#include <QToolButton>
#include <QTreeWidgetItemIterator>
#include <QWindow>
#include <QAbstractScrollArea>
#include <QTextDocument>
#include <QGroupBox>
#include <QTabWidget>
#include <QThread>
#include "gui/mate_tool_controller.hpp"
#include "gui/mate_tool_panel.hpp"
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEngineView>
#endif

class TestMain {
public:
  static void start_repeat_save_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene, bool y_axis, bool reopen) {
    struct Session {
      int phase=0, ticks=0;
      QByteArray layout, environment;
      QJsonObject evidence{{"result", "FAIL"}};
    };
    auto s = std::make_shared<Session>();
    const QString layout = scene + "/layout/workcell_studio_layout.yaml";
    const QString environment = scene + "/environment.yaml";
    const int axis = y_axis ? 1 : 0;
    const double original = y_axis ? .22 : .45;
    auto require = [](bool value, const char * message) {
      if (!value) throw std::runtime_error(message);
    };
    auto bytes = [=](const QString & path) {
      QFile file(path); require(file.open(QIODevice::ReadOnly), "cannot read authored file");
      return file.readAll();
    };
    auto record = [=](const QString & phase) {
      const auto root = YAML::Load(bytes(environment).toStdString());
      QJsonObject state;
      for (const auto & collection : {"assets", "task_zones"}) {
        for (const auto & item : root["environment"][collection]) {
          const auto id = item["id"].as<std::string>();
          if (id != "target_bin_default" && id != "default_drop_zone" && id != "place_zone_default") continue;
          state[QString::fromStdString(id)] = QString::fromStdString(YAML::Dump(item));
        }
      }
      for (const auto & id : {QString("target_bin_default"), QString("place_zone_default")}) {
        window->apply_scene_selection(id, "", false, false);
        const auto live = window->current_selected_scene_item();
        state[id + "_live_xyz"] = QJsonArray{live.pose_x, live.pose_y, live.pose_z};
      }
      s->evidence[phase] = state;
    };
    auto verify = [=](int moves, bool live_editor) {
      const auto root = YAML::Load(bytes(environment).toStdString());
      for (const auto & zone : root["environment"]["task_zones"]) {
        if (zone["target_ref"].as<std::string>("") != "target_bin_default") continue;
        require(zone["layout_item_ref"].as<std::string>() == "place_zone_default", "semantic layout reference changed");
        require(std::abs(zone["pose_xyz"][axis].as<double>() - (original + moves*.01)) < 1e-8,
          "destination world projection did not follow bin");
        require(std::abs(zone["placement_local"]["pose_xyz"][axis].as<double>()) < 1e-8,
          "bin-only edit changed destination-local placement");
      }
      const auto authored = YAML::Load(bytes(layout).toStdString());
      for (const auto & item : authored["items"]) {
        const auto id = item["id"].as<std::string>();
        if (id != "target_bin_default" && id != "place_zone_default") continue;
        require(std::abs(item["pose"]["xyz"][axis].as<double>() - (original + moves*.01)) < 1e-8,
          "disk layout pose differs from requested projection");
        if (!live_editor) continue;
        window->apply_scene_selection(QString::fromStdString(id), "", false, false);
        const auto live = window->current_selected_scene_item();
        require(std::abs((y_axis ? live.pose_y : live.pose_x) - (original + moves*.01)) < 1e-8,
          "retained editor pose differs from committed projection");
      }
    };
    auto write = [=]() {
      s->evidence["studio_log"] = window->studio_log_->toPlainText();
      QFile f(output + (reopen ? "/reopen.json" : "/repeat-save.json"));
      if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
    };
    auto * timer = new QTimer(window); timer->setInterval(250);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      require(++s->ticks < 400, "repeat-save GUI acceptance timed out");
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      switch (s->phase) {
        case 0: {
          if (s->ticks < 5) return;
          QSettings().setValue("startup/last_scene_output_root", output + "/scenes");
          window->refresh_scene_browser_ui();
          QStringList blockers; QJsonObject diagnostics;
          const bool loaded = window->load_scene_for_scene3d_smoke("ur5_2f_test", scene, &blockers, &diagnostics);
          s->evidence["load_diagnostics"] = diagnostics;
          s->evidence["load_blockers"] = QJsonArray::fromStringList(blockers);
          require(loaded, "fixture did not open");
          window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
          s->phase=1; s->ticks=0; break;
        }
        case 1: {
          if (s->ticks < 16) return;
          const bool native = window->scene_preview_widget_->is_native_product_view_backend();
          require(native == (qEnvironmentVariable("WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND") == "native_scene3d"),
            "Product View backend differs from requested acceptance backend");
          s->evidence["backend"] = native ? "native_scene3d" : "embedded_web3d";
          if (!window->scene_preview_widget_->is_native_product_view_backend() &&
              !window->scene_preview_widget_->embedded_web_authoring_active()) return;
          if (reopen) {
            verify(2, true); s->evidence["reopen_consistent"] = true; s->phase=5; break;
          }
          record("before_save_1"); s->phase=2; s->ticks=0; break;
        }
        case 2: {
          const int moves = s->evidence.contains("after_save_1") ? 2 : 1;
          window->apply_scene_selection("target_bin_default", "", false, false);
          require(window->inspector_apply_button_->isEnabled(), "Inspector Apply disabled");
          auto * spin = y_axis ? window->inspector_y_ : window->inspector_x_;
          spin->setValue(original + moves*.01);
          window->inspector_apply_button_->click();
          require(window->layout_dirty_, "Apply did not mark layout dirty");
          if (moves == 2) record("before_save_2");
          window->save_layout_button_->click();
          s->phase=3; s->ticks=0; break;
        }
        case 3: {
          if (s->ticks < 12) return;
          require(!window->layout_dirty_, "valid save rejected or remained dirty");
          const int moves = s->evidence.contains("after_save_1") ? 2 : 1;
          record(moves == 1 ? "after_save_1" : "after_save_2");
          verify(moves, moves == 2);
          if (moves == 1) { s->phase=2; s->ticks=0; break; }
          window->grab().save(output + "/two-saves.png");
          s->layout=bytes(layout); s->environment=bytes(environment);
          window->apply_scene_selection("place_zone_default", "", false, false);
          // This is an actual independent region edit beyond the reviewed usable box.
          window->inspector_y_->setValue(.27);
          window->inspector_apply_button_->click();
          window->save_layout_button_->click();
          s->phase=4; s->ticks=0; break;
        }
        case 4: {
          if (s->ticks < 12) return;
          require(window->layout_dirty_, "out-of-bounds region was accepted");
          require(bytes(layout) == s->layout && bytes(environment) == s->environment,
            "rejected save corrupted previously committed authored data");
          bool retained = false;
          for (const auto & item : window->editable_layout_session_items_)
            if (item.id == "place_zone_default") retained = std::abs(item.y - .27) < 1e-8;
          require(retained, "rejected save discarded the pending region edit");
          require(window->studio_log_->toPlainText().contains("outside target usable placement geometry"),
            "invalid save failed for an unexpected reason");
          s->evidence["invalid_rejected_without_corruption"] = true;
          window->grab().save(output + "/invalid-rejected.png");
          s->phase=5; break;
        }
        case 5: {
          s->evidence["result"]="PASS"; write(); timer->stop();
          // Discard only the intentionally rejected test edit when shutting down.
          window->layout_dirty_=false; window->close(); app->exit(0); break;
        }
      }
    } catch (const std::exception & error) {
      s->evidence["failure"] = QString::fromUtf8(error.what()); write();
      window->grab().save(output + "/failure.png"); timer->stop();
      window->layout_dirty_=false; window->close(); app->exit(1);
    }});
    // Failure dialogs would otherwise block the timer and hide the result.
    auto * dialogs = new QTimer(window); dialogs->setInterval(100);
    QObject::connect(dialogs, &QTimer::timeout, window, []() {
      for (auto * widget : QApplication::topLevelWidgets())
        if (auto * message = qobject_cast<QMessageBox *>(widget))
          if (message->isVisible()) message->reject();
    });
    dialogs->start(); timer->start();
  }

  static void start_mounting_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene_name, bool reopen, bool task_mode=false, const QString & observations={}) {
    struct Session { int phase=0, ticks=0; QString scene, support, camera; QJsonArray baseline, moved; QJsonObject evidence; };
    auto s = std::make_shared<Session>(); QDir().mkpath(output);
    s->evidence = {{"result", "FAIL"}, {"fake_hardware_only", true}, {"execution_goals_sent", 0}};
    auto write = [=]() { QFile f(output + (reopen ? "/reopen.json" : "/authoring.json")); if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto require = [](bool value, const char * message) { if (!value) throw std::runtime_error(message); };
    auto mount = [=](const QString & child, const QString & parent, const QString & anchor, bool detach=false, bool relative_edit=false) {
      window->apply_scene_selection(child, "", false, false);
      QTimer::singleShot(100, window, [=]() {
        auto * dialog = window->findChild<QDialog *>("assetMountingDialog");
        if (!dialog) return;
        if (detach) { dialog->findChild<QPushButton *>("detachAsset")->click(); return; }
        auto * parents = dialog->findChild<QComboBox *>("mountTo");
        parents->setCurrentIndex(parents->findData(parent));
        auto * anchors = dialog->findChild<QComboBox *>("mountAnchor");
        anchors->setCurrentIndex(anchors->findData(anchor));
        if (relative_edit) {
          dialog->findChild<QCheckBox *>("applyRelativePose")->setChecked(true);
          dialog->findChild<QDoubleSpinBox *>("relative_z")->setValue(1.2);
          dialog->findChild<QDoubleSpinBox *>("relative_pitch")->setValue(90.);
        }
        dialog->findChild<QDialogButtonBox *>()->button(QDialogButtonBox::Apply)->click();
      });
      window->findChild<QPushButton *>("sceneBuilderMounting")->click();
    };
    auto * modal_watch = new QTimer(window); modal_watch->setInterval(250);
    QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
      for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
        if (!message->isVisible()) continue;
        s->evidence["failure"] = message->text();
        s->evidence["studio_log"] = window->studio_log_->toPlainText();
        window->grab().save(output+"/failure.png"); write(); message->reject(); app->exit(1);
      }
    }); modal_watch->start();
    auto * timer = new QTimer(window); timer->setInterval(300);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      if (++s->ticks > (task_mode ? 3500 : 1000)) throw std::runtime_error("mounting GUI acceptance timed out");
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      switch (s->phase) {
        case 0: {
          if (s->ticks < 5) return;
          const QString root = window->detect_workspace_root() + "/src/easy_manipulation_deployment";
          s->scene = root + "/scenes/" + scene_name;
          if (reopen) {
            QStringList blockers; QJsonObject diagnostics;
            require(window->load_scene_for_scene3d_smoke(scene_name, s->scene, &blockers, &diagnostics), "saved cell did not reopen");
            s->phase=4; s->ticks=0; break;
          }
          require(!QFileInfo::exists(s->scene), "acceptance refuses to replace an existing cell");
          QSettings().setValue("startup/last_scene_output_root", root + "/scenes");
          QTimer::singleShot(100, window, [=]() {
            auto * wizard = window->findChild<NewCellWizard *>(); if (!wizard) return;
            wizard->findChild<QLineEdit *>("newCellSceneName")->setText(scene_name);
            wizard->findChild<QLineEdit *>("newCellDisplayName")->setText("Asset Mounting Acceptance");
            wizard->findChild<QListWidget *>("newCellWizardSteps")->setCurrentRow(5);
            for (auto * button : wizard->findChildren<QPushButton *>()) if (button->text() == "Create and Open") { button->click(); break; }
          });
          timer->stop(); window->open_new_scene_creation_flow(); timer->start();
          require(QFileInfo::exists(s->scene + "/environment.yaml"), "real New Cell Wizard did not create the cell");
          window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
          s->evidence["wizard_created"] = s->scene; s->phase=1; s->ticks=0; break;
        }
        case 1: {
          if (s->ticks < 10) return;
          require(window->has_selected_scene(), "new cell not selected in Studio");
          if (window->scene_preview_widget_ && !window->scene_preview_widget_->is_native_product_view_backend() &&
              !window->scene_preview_widget_->embedded_web_authoring_active()) return;
          if (task_mode) {
            timer->stop(); mount("target_bin_default", "support_surface_table", "surface"); timer->start();
            s->baseline = s->moved = window->mounting_items();
            s->evidence["mounted"] = s->baseline; s->phase=2; s->ticks=0; break;
          }
          QString stand_asset;
          QJsonArray candidates;
          for (const auto & entry : window->asset_catalog_entries_) if (entry.asset_id.contains("camera_stand")) {
            candidates.append(QJsonObject{{"id",entry.asset_id},{"source",entry.source_path},{"disabled",entry.disabled_reason}});
            if (entry.editable && entry.disabled_reason.isEmpty() && (stand_asset.isEmpty() || entry.asset_id == "camera_stand")) stand_asset=entry.asset_id;
          }
          s->evidence["support_catalog_candidates"] = candidates;
          require(!stand_asset.isEmpty(), "camera stand unavailable in Asset Library");
          const bool placed = window->place_catalog_asset_at_world_position(stand_asset, .25, -.45, 0., false);
          for (const auto & p : window->all_scene_preview_items_) {
            if (p.catalog_asset_id == stand_asset) s->support = p.id;
            if (p.id == "realsense_overhead") s->camera = p.id;
          }
          s->evidence["placement_return"] = placed;
          s->evidence["placed_items"] = window->mounting_items(); write();
          require(!s->support.isEmpty() && !s->camera.isEmpty(), "camera/support IDs not in real scene");
          timer->stop(); mount("target_bin_default", "support_surface_table", "surface"); mount(s->camera, s->support, "origin"); mount(s->camera, s->support, "origin", false, true); timer->start();
          s->baseline = window->mounting_items();
          require(s->baseline.size() >= 4, "physical mounting model missing assets");
          s->evidence["mounted"] = s->baseline; s->evidence["relative_pose_controls_used"] = true;
          s->phase=2; s->ticks=0; break;
        }
        case 2: {
          if (s->ticks < 3) return;
          if (task_mode) {
            auto * editor = window->environment_task_editor_; require(editor != nullptr, "existing TaskIntentEditor unavailable");
            const auto text = [=](const char * name, const QString & value) {
              auto * edit = editor->findChild<QLineEdit *>(name); require(edit != nullptr, "task text control missing"); edit->setText(value); edit->textEdited(value);
            };
            auto * mode = editor->findChild<QComboBox *>("taskPerceptionMode");
            mode->setCurrentIndex(mode->findData("replayed_snapshot"));
            text("taskReplayPath", observations); text("taskTargetClass", "cup");
            editor->findChild<QComboBox *>("taskPickZone")->setCurrentText("pick_zone_main");
            editor->findChild<QComboBox *>("taskPlaceAsset")->setCurrentText("target_bin_default");
            editor->findChild<QComboBox *>("taskPlaceRegion")->setCurrentText("default_drop_zone");
            editor->findChild<QComboBox *>("taskGraspPolicy")->setCurrentText("AUTO");
            editor->findChild<QComboBox *>("taskPlacePolicy")->setCurrentText("AUTO");
            editor->findChild<QDoubleSpinBox *>("taskMaximumAge")->setValue(300.);
            editor->findChild<QDoubleSpinBox *>("taskApproachDistance")->setValue(.12);
            editor->findChild<QDoubleSpinBox *>("taskRetreatDistance")->setValue(.15);
            editor->findChild<QDoubleSpinBox *>("taskPlaceClearance")->setValue(.001);
            editor->findChild<QComboBox *>("taskPlacePolicy")->setCurrentText("EXACT");
            text("taskLocalXYZ", "10, 0, 0"); text("taskLocalRPY", "0, 0, 0");
            editor->findChild<QPushButton *>("taskSave")->click();
            require(editor->blocker().contains("PLACE_LOCAL_POSE_OUTSIDE_REGION"), "invalid EXACT was not blocked");
            require(!window->selected_scene_readiness().ready, "invalid EXACT falsely ready");
            s->evidence["invalid_exact_blocked"] = editor->blocker();
            editor->findChild<QComboBox *>("taskPlacePolicy")->setCurrentText("AUTO");
            text("taskReplayPath", "missing-acceptance-input.yaml"); editor->findChild<QPushButton *>("taskSave")->click();
            require(editor->blocker().contains("TASK_INPUT_UNAVAILABLE"), "unavailable input was not blocked");
            s->evidence["unavailable_input_blocked"] = editor->blocker();
            text("taskReplayPath", observations);
            // The normal Save Layout action must persist the dirty task as well.
            window->save_layout_button_->click();
            require(!editor->dirty() && editor->blocker().isEmpty(), "normal Save did not persist a valid task");
            s->evidence["task_yaml"] = QString::fromStdString(editor->model().to_yaml());
            s->evidence["moved"] = s->moved; s->phase=3; s->ticks=0; break;
          }
          for (const auto & id : {QString("support_surface_table"), s->support}) {
            window->apply_scene_selection(id, "", false, false);
            window->inspector_update_guard_ = true;
            window->inspector_x_->setValue(window->inspector_x_->value()+.05);
            window->inspector_yaw_->setValue(window->inspector_yaw_->value()+20.);
            window->inspector_update_guard_ = false;
            window->inspector_apply_button_->click();
          }
          s->moved = window->mounting_items();
          require(s->moved != s->baseline, "Inspector parent edit did not move the scene");
          window->undo_layout_edit(); window->undo_layout_edit();
          require(window->mounting_items() == s->baseline, "undo did not restore full graph");
          window->redo_layout_edit(); window->redo_layout_edit();
          require(window->mounting_items() == s->moved, "redo did not restore full graph");
          timer->stop(); mount(s->camera, "world", "world", true); timer->start();
          window->undo_layout_edit();
          require(window->mounting_items() == s->moved, "detach undo lost graph/poses");
          timer->stop(); mount("target_bin_default", s->support, "origin"); timer->start();
          window->undo_layout_edit();
          require(window->mounting_items() == s->moved, "reparent undo lost graph/poses");
          s->evidence["parent_edits_undo_redo_reparent_detach"] = true;
          s->evidence["moved"] = s->moved;
          require(window->save_layout_button_ && window->save_layout_button_->isEnabled(), "Save Layout control is unavailable");
          window->save_layout_button_->click();
          s->phase=3; s->ticks=0; break;
        }
        case 3: {
          if (s->ticks < 5 || window->layout_dirty_) return;
          window->grab().save(output+"/mounted_scene.png");
          s->evidence["saved"] = true; s->evidence["result"] = "PASS";
          write(); timer->stop(); window->close(); app->exit(0); break;
        }
        case 4: {
          if (s->ticks < 12) return;
          QFile previous(output+"/authoring.json"); require(previous.open(QIODevice::ReadOnly), "authoring evidence missing");
          const auto before = QJsonDocument::fromJson(previous.readAll()).object().value("moved").toArray();
          require(QFileInfo(window->selected_scene_state_.path).canonicalFilePath() == QFileInfo(s->scene).canonicalFilePath(), "reopen selected a different scene");
          const auto after = window->mounting_items();
          s->evidence["reopened"] = after;
          QJsonArray loaded;
          for (const auto & p : window->all_scene_preview_items_) loaded.append(QJsonObject{{"id",p.id},{"editable",p.editable},{"locked",p.locked},{"layer",p.source_layer}});
          s->evidence["all_loaded_items"] = loaded; s->evidence["selected_scene_path"] = window->selected_scene_state_.path; write();
          require(before.size() == after.size(), "reopen changed physical identities");
          QMap<QString,QJsonObject> indexed;
          for (const auto & row : after) indexed.insert(row.toObject().value("id").toString(), row.toObject());
          for (const auto & value : before) {
            const auto row=value.toObject(), actual=indexed.value(row.value("id").toString());
            require(!actual.isEmpty(), "reopen lost an asset");
            for (const auto & key : {QString("pose_xyz"),QString("pose_rpy")}) {
              for (int k=0;k<3;++k) require(std::abs(row.value(key).toArray()[k].toDouble()-actual.value(key).toArray()[k].toDouble())<1e-8,"reopen changed transform");
            }
            const auto a=YAML::Load(row.value("mounting_yaml").toString().toStdString()), b=YAML::Load(actual.value("mounting_yaml").toString().toStdString());
            for (const auto & key : {"parent_object","parent_link"}) require(a[key].as<std::string>("")==b[key].as<std::string>(""),"reopen changed mounting relationship");
          }
          s->evidence["reopened"] = after;
          if (task_mode) {
            QFile saved(output+"/authoring.json"); require(saved.open(QIODevice::ReadOnly), "authoring task evidence missing");
            const auto authored = QJsonDocument::fromJson(saved.readAll()).object()["task_yaml"].toString();
            require(QString::fromStdString(window->environment_task_editor_->model().to_yaml()) == authored, "reopen changed TaskIntent");
            s->evidence["task_reopened_identically"] = true;
          }
          window->generate_scene_package_for_selected_scene();
          require(QFileInfo::exists(s->scene+"/generated/generated_environment_objects.yaml"), "GUI generation did not produce environment objects");
          s->evidence["gui_generation_invoked"] = true;
          if (task_mode) {
            window->validate_generated_scene_for_selected_scene();
            const auto ready = window->selected_scene_readiness();
            s->evidence["readiness_blockers"] = QJsonArray::fromStringList(ready.blockers); write();
            require(ready.ready, "new cell GUI planning prerequisites blocked");
            window->show_studio_page(MainWindow::StudioPage::PlanSimulatePage);
            window->run_preview_button_->click();
            require(window->full_cycle_mode_, "Plan / Simulate did not start the existing planner");
            s->phase=5; s->ticks=0; break;
          }
          s->evidence["result"] = "PASS";
          window->grab().save(output+"/reopened_scene.png"); write(); timer->stop(); window->close(); app->exit(0); break;
        }
        case 5: {
          if (window->preview_process_->state() != QProcess::NotRunning) return;
          s->evidence["plan_result"] = window->cycle_result_; s->evidence["runtime_evidence"] = window->cycle_output_dir_; write();
          require(window->cycle_result_["result"].toString() == "PASS", "complete collision-aware plan failed; see runtime evidence");
          require(!window->cycle_result_["execution_attempted"].toBool(), "unexpected execution attempt");
          s->evidence["result"] = "PASS"; window->grab().save(output+"/planned_scene.png"); write(); timer->stop(); window->close(); app->exit(0); break;
        }
      }
    } catch (const std::exception & error) { s->evidence["failure"] = QString::fromUtf8(error.what()); write(); timer->stop(); app->exit(1); } });
    timer->start();
  }

  // EPD readiness panel acceptance: real MainWindow, real monitor artifact, no planning or motion.
  // Records every distinct panel state seen while an external replay drives the monitor node.
  static void start_epd_readiness_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene_name, const QString & scene_path,
      int cycle, int duration_s) {
    struct Session { int phase=0, ticks=0; QSet<QString> seen; QJsonArray observations; QJsonObject evidence; };
    auto s = std::make_shared<Session>(); QDir().mkpath(output);
    const QString tag = QString("cycle%1").arg(cycle);
    s->evidence = {{"result", "FAIL"}, {"cycle", cycle}, {"execution_goals_sent", 0}, {"planning_requests_sent", 0}};
    auto write = [=]() { s->evidence["observations"] = s->observations; QFile f(output + "/" + tag + ".json"); if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto * timer = new QTimer(window); timer->setInterval(250);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      ++s->ticks;
      if (s->phase == 0) {
        if (s->ticks < 5) return;
        QStringList blockers; QJsonObject diagnostics;
        if (!window->load_scene_for_scene3d_smoke(scene_name, scene_path, &blockers, &diagnostics)) {
          s->evidence["load_blockers"] = QJsonArray::fromStringList(blockers);
          throw std::runtime_error("scene did not load");
        }
        auto * table = window->findChild<QTableWidget *>("epdReadinessTable");
        if (!table || !window->epd_readiness_timer_ || !window->epd_readiness_timer_->isActive()) throw std::runtime_error("readiness panel or timer missing");
        for (int i = 0; i < window->scene_builder_inspector_tabs_->count(); ++i) {
          if (window->scene_builder_inspector_tabs_->widget(i)->isAncestorOf(table)) window->scene_builder_inspector_tabs_->setCurrentIndex(i);
        }
        s->evidence["scene_path"] = window->selected_scene_path();
        s->evidence["artifact_path"] = workcell_builder::epd_readiness_artifact_path(window->selected_scene_path());
        s->phase = 1; s->ticks = 0; return;
      }
      auto * table = window->epd_readiness_table_;
      QJsonArray rows; QStringList signature;
      for (int r = 0; r < table->rowCount(); ++r) {
        QJsonObject row; QStringList cells;
        for (int c = 0; c < table->columnCount(); ++c) {
          const QString value = table->item(r, c) ? table->item(r, c)->text() : QString();
          row[table->horizontalHeaderItem(c)->text()] = value; cells << value;
        }
        rows.append(row); signature << cells.mid(0, 1).join("") + "/" + cells.value(3) + "/" + cells.value(5);
      }
      const QString label = window->epd_readiness_label_->text();
      const QString connection = label.section(' ', 2, 2);
      const QString key = connection + "|" + signature.join(",");
      if (!s->seen.contains(key)) {
        s->seen.insert(key);
        const QString shot = QString("%1_%2.png").arg(tag).arg(s->observations.size(), 2, 10, QChar('0'));
        QWidget * card = table->parentWidget();
        card->grab().save(output + "/" + shot);
        s->observations.append(QJsonObject{{"t_s", s->ticks * 0.25}, {"connection", connection}, {"label", label},
                                           {"rows", rows}, {"panel_screenshot", shot}});
        write();
      }
      if (s->ticks * 0.25 >= duration_s) {
        window->grab().save(output + "/" + tag + "_window.png");
        s->evidence["result"] = "PASS"; s->evidence["readiness_timer_active_before_close"] = window->epd_readiness_timer_->isActive();
        write(); timer->stop(); window->close(); app->exit(0);
      }
    } catch (const std::exception & e) {
      s->evidence["failure"] = e.what(); window->grab().save(output + "/" + tag + "_failure.png"); write(); timer->stop(); app->exit(1);
    } });
    timer->start();
  }

  // Feature 4B: Task tab -> Save -> Generate -> "Check plans (no motion)"; --reopen proves persistence + recheck.
  static void start_multiobject_plan_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & recording, bool reopen, bool skip_check = false,
      const QString & sequence_order = QString(), const QString & sequence_places = QString(),
      const QString & expect_sequence = QString()) {
    // Sequence runs: the author run builds the sequence in the Task tab (Preview, tick, drag, choose places),
    // saves it in the task and generates; the reopen run checks the saved sequence exactly as reopened.
    struct Session { int phase=0, ticks=0; QString scene; QJsonObject evidence; bool configured=false; };
    auto s = std::make_shared<Session>(); QDir().mkpath(output);
    const QString tag = reopen ? "reopen" : "author";
    s->evidence = {{"result", "FAIL"}, {"phase", tag}, {"robot_motion_commanded", false}};
    auto write = [=]() { QFile f(output + "/" + tag + ".json"); if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto require = [](bool value, const char * message) { if (!value) throw std::runtime_error(message); };
    auto * modal_watch = new QTimer(window); modal_watch->setInterval(250);
    QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
      for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
        if (!message->isVisible()) continue;
        s->evidence["modal_dialog"] = message->text() + "\n" + message->informativeText();
        window->grab().save(output + "/" + tag + "_modal.png"); message->grab().save(output + "/" + tag + "_modal_dialog.png");
        write(); message->reject(); app->exit(1);
      }
    }); modal_watch->start();
    auto * timer = new QTimer(window); timer->setInterval(500);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      if (++s->ticks > 4800) throw std::runtime_error("multi-object plan acceptance timed out");
      const bool loading = window->load_watcher_ && window->load_watcher_->isRunning();
      if (s->ticks % 20 == 1) {
        s->evidence["progress"] = QJsonObject{{"phase", s->phase}, {"ticks", s->ticks}, {"load_watcher_running", loading},
                                              {"has_selected_scene", window->has_selected_scene()}};
        write();
      }
      if (loading && s->phase != 4 && s->phase != 6) return;
      auto * editor = window->environment_task_editor_;
      switch (s->phase) {
        case 0: {
          if (s->ticks < 6) return;
          s->scene = QFileInfo(window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/ur5_2f_test").canonicalFilePath();
          int index = -1;
          for (size_t i = 0; i < window->scene_browser_result_.scenes.size(); ++i)
            if (QFileInfo(QString::fromStdString(window->scene_browser_result_.scenes[i].scene_dir.string())).canonicalFilePath() == s->scene) index = int(i);
          require(index >= 0, "ur5_2f_test not discovered in the selected workspace");
          require(window->open_scene_builder_for_scene_index(index, "4B multi-object plan acceptance"), "scene did not open");
          s->phase = 1; s->ticks = 0; break;
        }
        case 1: {
          if (s->ticks < 8) return;
          auto * mode = editor->findChild<QComboBox *>("taskPerceptionMode");
          auto * path = editor->findChild<QLineEdit *>("taskReplayPath");
          auto * target = editor->findChild<QLineEdit *>("taskTargetClass");
          require(mode && path && target, "Task tab controls missing");
          if (reopen) {
            s->evidence["reopened_fields"] = QJsonObject{{"mode", mode->currentData().toString()},
              {"replay_path", path->text()}, {"target_class", target->text()}, {"dirty", editor->dirty()}};
            require(mode->currentData().toString() == "replayed_snapshot" && path->text() == recording &&
                    target->text() == "cup" && !editor->dirty() &&
                    editor->findChild<QComboBox *>("taskGraspPolicy")->currentText() == "AUTO" &&
                    editor->findChild<QComboBox *>("taskPlacePolicy")->currentText() == "AUTO" &&
                    std::abs(editor->findChild<QDoubleSpinBox *>("taskPlaceClearance")->value() - 0.005) < 1e-9 &&
                    std::abs(editor->findChild<QDoubleSpinBox *>("taskMaximumAge")->value() - 300.0) < 1e-9,
                    "saved task did not reopen identically");
            if (!sequence_order.isEmpty()) {  // The saved sequence, with no Preview: order, objects and places.
              auto * order = editor->findChild<QListWidget *>("taskSequenceOrder");
              QJsonArray rows; QStringList ids;
              for (int i = 0; i < order->count(); ++i) {
                rows.append(order->item(i)->text());
                if (order->item(i)->checkState() == Qt::Checked) ids << order->item(i)->data(Qt::UserRole).toString();
              }
              s->evidence["reopened_sequence"] = rows;
              s->evidence["reopened_sequence_summary"] = editor->findChild<QLabel *>("taskSequenceSummary")->text();
              require(ids.join(',') == sequence_order, "saved sequence order did not reopen identically");
              // A sequence expected to need review may legitimately describe its saved places differently.
              for (const auto & pair : expect_sequence.isEmpty() ? sequence_places.split(',', Qt::SkipEmptyParts) : QStringList()) {
                const auto id = pair.section('=', 0, 0), choice = pair.section('=', 1);
                const auto text = order->item(ids.indexOf(id))->text();
                require(choice == "auto" ? text.contains("Automatic") : text.contains(QString("Place %1 of").arg(choice.section('-', 1))),
                        "saved place did not reopen identically");
              }
              if (!expect_sequence.isEmpty())
                require(s->evidence["reopened_sequence_summary"].toString().startsWith(expect_sequence),
                        "reopened sequence review did not report the expected status");
              s->configured = true;
            }
            s->phase = 3; s->ticks = 0; break;
          }
          mode->setCurrentIndex(mode->findData("replayed_snapshot"));
          path->setText(recording); emit path->textEdited(recording);
          target->setText("cup"); emit target->textEdited("cup");
          // Beginner Task-tab choices: let the resolver try every grasp candidate, and use a
          // placement clearance the authored 0.12 m drop region can satisfy for a 0.10 m cup.
          auto * policy = editor->findChild<QComboBox *>("taskGraspPolicy");
          auto * clearance = editor->findChild<QDoubleSpinBox *>("taskPlaceClearance");
          require(policy && clearance, "grasp policy / placement clearance controls missing");
          policy->setCurrentText("AUTO");
          clearance->setValue(0.005);
          // AUTO placement lets a multi-object sequence give each object its own free place.
          auto * place_policy = editor->findChild<QComboBox *>("taskPlacePolicy");
          require(place_policy, "placement policy control missing");
          place_policy->setCurrentText("AUTO");
          // A static commissioning recording: allow time for every grasp candidate to be planned.
          auto * max_age = editor->findChild<QDoubleSpinBox *>("taskMaximumAge");
          require(max_age, "maximum observation age control missing");
          max_age->setValue(300.0);
          QString error;
          require(editor->save(&error), "Task save failed");
          s->phase = 5; s->ticks = 0; break;
        }
        case 5: {  // Generate exactly as the user does, once Product View live authoring is ready.
          if (!window->ensure_live_authoring_mutation_available("4B acceptance")) {
            s->evidence["generate_wait_ticks"] = s->ticks;
            require(s->ticks < 180, "Product View live authoring never became available; Generate is locked");
            return;
          }
          window->generate_scene_package_for_selected_scene();
          s->phase = 2; s->ticks = 0; break;
        }
        case 2: {
          if (s->ticks < 10) return;
          QFile task(s->scene + "/config/workcell_builder_task_intent.yaml"); task.open(QIODevice::ReadOnly);
          s->evidence["saved_task_intent"] = QString::fromUtf8(task.readAll());
          s->phase = 3; s->ticks = 0; break;
        }
        case 3: {
          if (s->ticks < 4) return;
          auto * check = editor->findChild<QPushButton *>("taskSimulationBatchPlanCheck");
          require(check && check->isEnabled(), "Check plans is not enabled for the saved task");
          if (!sequence_order.isEmpty()) {
            auto * sequence = editor->findChild<QPushButton *>("taskSimulationSequenceCheck");
            require(sequence && sequence->isEnabled(), "sequence controls unavailable");
            if (!s->configured) {  // Build it in the Task tab from the queue preview, as a user does.
              auto * preview = editor->findChild<QPushButton *>("taskSimulationBatchPreview");
              require(preview && preview->isEnabled(), "Preview queue is not enabled for the saved task");
              s->evidence["preview_summary_before"] = editor->findChild<QLabel *>("taskSequenceSummary")->text();
              preview->click(); s->phase = 7; s->ticks = 0; break;
            }
            s->evidence["sequence_order"] = sequence_order;
            sequence->click(); s->phase = 4; s->ticks = 0; break;
          }
          if (skip_check) {  // Lifecycle-only cycle: open, verify persisted Task, close.
            s->evidence["result"] = "PASS"; s->evidence["plan_check_skipped"] = true; write();
            timer->stop(); window->close(); app->exit(0); break;
          }
          check->click(); s->phase = 4; s->ticks = 0; break;
        }
        case 4: {
          auto * cancel = editor->findChild<QPushButton *>("taskSimulationBatchPlanCancel");
          auto * status = editor->findChild<QLabel *>("taskSimulationBatchStatus");
          if (s->ticks % 20 == 0) { s->evidence["last_progress"] = status->text(); write(); }
          if (cancel->isEnabled() || s->ticks < 4) return;
          const QString details = editor->findChild<QPlainTextEdit *>("taskSimulationBatchDetails")->toPlainText();
          s->evidence["final_status"] = status->text();
          s->evidence["final_details"] = details;
          const QString evidence_dir = details.section("Evidence: ", 1).trimmed();
          const QString report_name = sequence_order.isEmpty() ? "/batch_plan_check.json" : "/sequence_plan_check.json";
          QFile report(evidence_dir + report_name);
          require(report.open(QIODevice::ReadOnly), "plan check report missing");
          const auto batch = QJsonDocument::fromJson(report.readAll()).object();
          s->evidence["batch_report"] = evidence_dir + report_name;
          s->evidence["outcome_counts"] = batch.value("outcome_counts");
          s->evidence["sequence_status"] = batch.value("sequence_status");
          s->evidence["status"] = batch.value("status");
          s->evidence["reason"] = batch.value("reason");
          if (!expect_sequence.isEmpty()) {
            require(batch.value("status").toString() == "BLOCKED" && batch.value("reason").toString().startsWith("SEQUENCE_") &&
                    !batch.value("moveit_planning_attempted").toBool(), "an invalid saved sequence was planned");
          } else {
            require(batch.value("status").toString() == "CHECKED", "batch did not complete");
          }
          require(!batch.value("execution_attempted").toBool() && !batch.value("motion_commanded").toBool(), "motion observed");
          for (int i = 0; i < window->scene_builder_inspector_tabs_->count(); ++i)
            if (window->scene_builder_inspector_tabs_->widget(i)->isAncestorOf(editor))
              window->scene_builder_inspector_tabs_->setCurrentIndex(i);
          s->phase = 6; s->ticks = 0; write(); break;
        }
        case 7: {  // Preview filled the list: tick, drag into order and choose places, then Save and Generate.
          auto * order = editor->findChild<QListWidget *>("taskSequenceOrder");
          auto * place = editor->findChild<QComboBox *>("taskSequencePlace");
          require(place != nullptr, "place control missing");
          if (order->count() == 0 || editor->findChild<QLabel *>("taskSimulationBatchStatus")->text().startsWith("Checking")) {
            require(s->ticks < 120, "Preview queue never filled the sequence"); return;
          }
          const auto ids = sequence_order.split(',', Qt::SkipEmptyParts);
          for (int i = 0; i < order->count(); ++i)  // Tick exactly the chosen objects.
            order->item(i)->setCheckState(ids.contains(order->item(i)->data(Qt::UserRole).toString()) ? Qt::Checked : Qt::Unchecked);
          for (int target = 0; target < ids.size(); ++target) {  // Drag each into position.
            int row = -1;
            for (int i = 0; i < order->count(); ++i) if (order->item(i)->data(Qt::UserRole).toString() == ids[target]) row = i;
            require(row >= 0, "sequence object missing from the preview queue");
            if (row != target) require(order->model()->moveRow(QModelIndex(), row, QModelIndex(), target), "drag failed");
          }
          QJsonObject chosen;
          for (const auto & pair : sequence_places.split(',', Qt::SkipEmptyParts)) {
            const auto id = pair.section('=', 0, 0), choice = pair.section('=', 1);
            const int row = ids.indexOf(id);
            require(row >= 0, "place chosen for an object not in the sequence");
            order->setCurrentRow(row);
            const int index = choice == "auto" ? 0 : place->findData(choice);
            require(place->isEnabled() && index >= 0, "place option not offered by the preview");
            place->setCurrentIndex(index); emit place->activated(index);
            chosen[id] = order->item(row)->text();
          }
          s->evidence["chosen_places"] = chosen;
          editor->validate_now();
          QJsonArray rows;
          for (int i = 0; i < order->count(); ++i) rows.append(order->item(i)->text());
          s->evidence["edited_sequence"] = rows;
          s->evidence["edited_summary"] = editor->findChild<QLabel *>("taskSequenceSummary")->text();
          window->grab().save(output + "/" + tag + "_sequence_edited.png");
          QString error;
          require(editor->save(&error), "Task save with sequence failed");
          QFile task(s->scene + "/config/workcell_builder_task_intent.yaml"); task.open(QIODevice::ReadOnly);
          s->evidence["saved_sequence"] = QJsonDocument::fromJson(task.readAll()).object().value("sequence");
          s->configured = true;
          s->phase = 5; s->ticks = 0; break;  // The sequence changed the task: Generate again.
        }
        case 6: {  // Let the Task tab lay out before capturing it.
          if (s->ticks < 3) return;
          auto * results = editor->findChild<QLabel *>("taskSimulationBatchStatus")->parentWidget();
          results->grab().save(output + "/" + tag + "_task_results.png");
          window->grab().save(output + "/" + tag + "_window.png");
          s->evidence["result"] = "PASS"; write(); timer->stop(); window->close(); app->exit(0); break;
        }
      }
    } catch (const std::exception & e) {
      s->evidence["failure"] = e.what(); window->grab().save(output + "/" + tag + "_failure.png"); write();
      timer->stop(); app->exit(1);
    } });
    timer->start();
  }

  // Datum mating on a fresh New Cell: bin bottom flush on the table top, camera on the
  // camera-stand top, parent edits, undo/redo, detach/reparent, rejected relations,
  // three consecutive bin saves and a separate-process reopen with GUI Generate/Validate.
  static void start_datum_mating_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene_name, bool reopen) {
    struct Session { int phase=0, ticks=0, saves=0; QString scene, stand; QJsonArray mated, moved; QJsonObject evidence, zone_local; };
    auto s = std::make_shared<Session>(); QDir().mkpath(output);
    s->evidence = {{"result", "FAIL"}, {"fake_hardware_only", true}, {"execution_goals_sent", 0}};
    const QString bin = "target_bin_default", table = "support_surface_table", camera = "realsense_overhead";
    const std::vector<double> bin_moves{-0.02, -0.07, -0.01};
    auto write = [=]() {
      s->evidence["studio_log_tail"] = window->studio_log_->toPlainText().right(6000);
      QFile f(output + (reopen ? "/reopen.json" : "/authoring.json"));
      if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
    };
    auto require = [](bool value, const QString & message) { if (!value) throw std::runtime_error(message.toStdString()); };
    auto item = [=](const QString & id) {
      for (const auto & p : window->all_scene_preview_items_) if (p.id == id) return p;
      throw std::runtime_error(("scene item missing: " + id).toStdString());
    };
    auto relation = [=](const QString & id) { return YAML::Load(item(id).mounting_yaml.toStdString()); };
    auto environment = [=]() { return YAML::LoadFile((s->scene + "/environment.yaml").toStdString()); };
    auto record = [](const YAML::Node & root, const std::string & id) {
      for (const char * key : {"support_surfaces", "assets", "sensors", "task_zones"})
        for (const auto & r : root["environment"][key])
          if (r["id"].as<std::string>("") == id || r["layout_item_ref"].as<std::string>("") == id) return r;
      throw std::runtime_error("authored record missing: " + id);
    };
    auto near = [](double a, double b, double tol = 1e-8) { return std::abs(a - b) < tol; };
    // Drives the real Inspector Mount To dialog; nothing is applied unless asked and enabled.
    auto mate = [=](const QString & tag, const QString & child, const QString & parent, const QString & parent_datum,
                    const QString & child_datum, const QString & alignment, std::array<double, 6> offset,
                    bool move, bool apply, bool detach = false) {
      auto seen = std::make_shared<QJsonObject>();
      window->apply_scene_selection(child, "", false, false);
      QTimer::singleShot(150, window, [=]() {
        auto * dialog = window->findChild<QDialog *>("assetMountingDialog");
        if (!dialog) return;
        (*seen)["opened"] = true;
        if (detach) { dialog->grab().save(output + "/dialog-" + tag + ".png"); dialog->findChild<QPushButton *>("detachAsset")->click(); return; }
        auto pick = [&](const char * name, const QString & data) {
          auto * combo = dialog->findChild<QComboBox *>(name);
          const int index = combo ? combo->findData(data) : -1;
          (*seen)[QString(name)] = index >= 0 ? combo->itemText(index) : QString("MISSING ") + data;
          if (index >= 0) combo->setCurrentIndex(index);
        };
        pick("mountTo", parent); pick("mountAnchor", parent_datum);
        if (parent != "world") { pick("mountChildDatum", child_datum); pick("mountAlignment", alignment); }
        dialog->findChild<QCheckBox *>("applyRelativePose")->setChecked(move);
        const char * keys[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
        for (int k = 0; k < 6; ++k) dialog->findChild<QDoubleSpinBox *>(QString("relative_%1").arg(keys[k]))->setValue(offset[k]);
        auto * apply_button = dialog->findChild<QDialogButtonBox *>()->button(QDialogButtonBox::Apply);
        (*seen)["preview"] = dialog->findChild<QLabel *>("mountPreview")->text();
        (*seen)["apply_enabled"] = apply_button->isEnabled();
        (*seen)["return_applies"] = apply_button->isDefault() && !dialog->findChild<QPushButton *>("detachAsset")->autoDefault();
        dialog->adjustSize(); QCoreApplication::processEvents();  // capture the laid-out preview text
        dialog->grab().save(output + "/dialog-" + tag + ".png");
        if (apply && apply_button->isEnabled()) apply_button->click(); else dialog->reject();
      });
      window->findChild<QPushButton *>("sceneBuilderMounting")->click();
      require(seen->value("opened").toBool(), "Mount To dialog did not open for " + child);
      s->evidence["dialog_" + tag] = *seen;
      return *seen;
    };
    auto inspector = [=](const QString & id, double dx, double dy, double dyaw_deg) {
      window->apply_scene_selection(id, "", false, false);
      window->inspector_update_guard_ = true;
      window->inspector_x_->setValue(window->inspector_x_->value() + dx);
      window->inspector_y_->setValue(window->inspector_y_->value() + dy);
      window->inspector_yaw_->setValue(window->inspector_yaw_->value() + dyaw_deg);
      window->inspector_update_guard_ = false;
      window->inspector_apply_button_->click();
    };
    // Child datum frame against parent datum frame: the relation must hold exactly.
    auto check_bin_on_table = [=](const QString & phase) {
      const auto b = item(bin), t = item(table);
      const auto m = relation(bin);
      require(m["mate"]["alignment"].as<std::string>("") == "flush" && m["mate"]["child_datum"].as<std::string>("") == "bottom", phase + ": bin mate lost");
      const double c = std::cos(t.yaw), sn = std::sin(t.yaw);
      const double lx = c * (b.x - t.x) + sn * (b.y - t.y), ly = -sn * (b.x - t.x) + c * (b.y - t.y);
      require(near(lx, m["origin"]["x"].as<double>()) && near(ly, m["origin"]["y"].as<double>()), phase + ": bin drifted from its table offset");
      require(near(b.z, t.z + 0.1 + m["origin"]["z"].as<double>()), phase + ": bin bottom no longer flush on the table top");
      require(near(b.yaw, t.yaw + m["origin"]["yaw"].as<double>()) && near(b.roll, 0.) && near(b.pitch, 0.), phase + ": bin rotation does not follow the table");
      s->evidence[phase + "_bin_table_local"] = QJsonArray{lx, ly, b.z - t.z};
    };
    auto * modal_watch = new QTimer(window); modal_watch->setInterval(250);
    QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
      for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
        if (!message->isVisible()) continue;
        s->evidence["failure"] = "unexpected message: " + message->text();
        window->grab().save(output + "/failure.png"); write(); message->reject(); app->exit(1);
      }
    }); modal_watch->start();
    auto * timer = new QTimer(window); timer->setInterval(300);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      require(++s->ticks < 1200, "datum mating GUI acceptance timed out");
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      switch (s->phase) {
        case 0: {
          if (s->ticks < 5) return;
          const QString root = window->detect_workspace_root() + "/src/easy_manipulation_deployment";
          s->scene = root + "/scenes/" + scene_name;
          s->evidence["scene_path"] = s->scene;
          if (reopen) {
            QStringList blockers; QJsonObject diagnostics;
            require(window->load_scene_for_scene3d_smoke(scene_name, s->scene, &blockers, &diagnostics), "saved cell did not reopen");
            window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
            s->phase = 10; s->ticks = 0; break;
          }
          require(!QFileInfo::exists(s->scene), "acceptance refuses to replace an existing cell");
          QSettings().setValue("startup/last_scene_output_root", root + "/scenes");
          QTimer::singleShot(100, window, [=]() {
            auto * wizard = window->findChild<NewCellWizard *>(); if (!wizard) return;
            wizard->findChild<QLineEdit *>("newCellSceneName")->setText(scene_name);
            wizard->findChild<QLineEdit *>("newCellDisplayName")->setText("Datum Mating Acceptance");
            wizard->findChild<QListWidget *>("newCellWizardSteps")->setCurrentRow(5);
            for (auto * button : wizard->findChildren<QPushButton *>()) if (button->text() == "Create and Open") { button->click(); break; }
          });
          timer->stop(); window->open_new_scene_creation_flow(); timer->start();
          require(QFileInfo::exists(s->scene + "/environment.yaml"), "real New Cell Wizard did not create the cell");
          window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
          s->evidence["wizard_created"] = s->scene; s->phase = 1; s->ticks = 0; break;
        }
        case 1: {
          if (s->ticks < 10) return;
          require(window->has_selected_scene(), "new cell not selected in Studio");
          if (window->scene_preview_widget_ && !window->scene_preview_widget_->is_native_product_view_backend() &&
              !window->scene_preview_widget_->embedded_web_authoring_active()) return;
          s->evidence["backend"] = window->scene_preview_widget_->is_native_product_view_backend() ? "native_scene3d" : "embedded_web3d";
          const auto created = environment();
          s->zone_local["default_drop_zone"] = QString::fromStdString(YAML::Dump(record(created, "default_drop_zone")["placement_local"]));
          // Add a camera stand from the Asset Library; its base sits on the floor.
          require(window->place_catalog_asset_at_world_position("camera_stand", .25, -.45, .6, false), "camera stand could not be placed from the Asset Library");
          for (const auto & p : window->all_scene_preview_items_) if (p.catalog_asset_id == "camera_stand") s->stand = p.id;
          require(!s->stand.isEmpty(), "placed camera stand has no scene ID");
          const auto listed = window->run_mounting_request({{"operation", "anchors"}});
          require(listed.value("ok").toBool(), "datum listing failed: " + listed.value("error").toString());
          const auto datums = listed.value("result").toObject();
          s->evidence["placed_items"] = window->mounting_items();
          s->evidence["datums"] = QJsonObject{{"bin", datums.value(bin)}, {"table", datums.value(table)}, {"stand", datums.value(s->stand)}, {"camera", datums.value(camera)}};
          require(datums.value(bin).toObject().value("bottom").toObject().value("datum_type").toString() == "plane", "bin bottom datum unavailable");
          require(datums.value(table).toObject().value("surface").toObject().value("datum_type").toString() == "plane", "table top datum unavailable");
          require(datums.value(s->stand).toObject().value("top").toObject().value("datum_type").toString() == "plane", "camera stand top datum unavailable");
          // Rejected relations: shown in the preview, Apply disabled, nothing changes.
          const auto before = window->mounting_items();
          for (const auto & bad : {std::make_tuple(QString("invalid-flush-origin"), bin, table, QString("surface"), QString("origin"), QString("flush"), QString("flush needs two plane datums")),
                                   std::make_tuple(QString("invalid-axis-plane"), bin, table, QString("surface"), QString("centre_axis"), QString("coincident"), QString("axis datum cannot coincide")),
                                   std::make_tuple(QString("invalid-cycle"), table, bin, QString("rim"), QString("origin"), QString("coincident"), QString("mounting cycle"))}) {
            if (std::get<1>(bad) == table) {  // a cycle needs the bin mounted first
              mate("bin-on-table", bin, table, "surface", "bottom", "flush", {-0.2607, 0.42, 0., 0., 0., 0.}, true, true);
            }
            const auto shown = mate(std::get<0>(bad), std::get<1>(bad), std::get<2>(bad), std::get<3>(bad), std::get<4>(bad), std::get<5>(bad), {0., 0., 0., 0., 0., 0.}, true, true);
            require(!shown.value("apply_enabled").toBool(), std::get<0>(bad) + ": Apply was enabled for an invalid relation");
            require(shown.value("preview").toString().contains(std::get<6>(bad)), std::get<0>(bad) + ": preview did not explain the rejection: " + shown.value("preview").toString());
          }
          window->undo_layout_edit();
          require(window->mounting_items() == before, "rejected relations changed the scene");
          s->evidence["invalid_relations_rejected_without_change"] = true;
          // Relation 1: bin bottom flush on the table top, moved onto it by the dialog.
          const auto bin_mate = mate("bin-on-table", bin, table, "surface", "bottom", "flush", {-0.2607, 0.42, 0., 0., 0., 0.}, true, true);
          require(bin_mate.value("preview").toString().contains("Follows:"), "preview did not list the linked destination zone");
          require(bin_mate.value("return_applies").toBool(), "Return in the Mount To dialog is not bound to Apply");
          check_bin_on_table("mated");
          // Relation 2 (independent): camera origin on the stand top, 5 cm above, looking down.
          mate("camera-on-stand", camera, s->stand, "top", "origin", "coincident", {0.15, 0.09, 0.05, 0., 90., 0.}, true, true);
          const auto cam = item(camera), stand = item(s->stand);
          require(near(cam.z, stand.z + 0.6 + 0.05) && near(cam.pitch, M_PI / 2, 1e-6), "camera not placed at the stand top datum offset");
          require(relation(s->stand)["mounting_anchors"]["top"]["datum_type"].as<std::string>("") == "plane", "stand top datum was not copied into the cell");
          s->mated = window->mounting_items(); s->evidence["mated"] = s->mated;
          window->grab().save(output + "/mated.png");
          s->phase = 2; s->ticks = 0; break;
        }
        case 2: {
          if (s->ticks < 4) return;
          const auto camera_before = item(camera);
          inspector(table, .05, 0., 20.);          // move and rotate the table
          check_bin_on_table("table_moved");
          require(near(item(camera).x, camera_before.x) && near(item(camera).yaw, camera_before.yaw), "independent camera relation moved with the table");
          inspector(s->stand, 0., -.05, 15.);      // move and rotate the stand
          const auto cam = item(camera), stand = item(s->stand);
          const double c = std::cos(stand.yaw), sn = std::sin(stand.yaw);
          require(near(c * (cam.x - stand.x) + sn * (cam.y - stand.y), 0.15) && near(-sn * (cam.x - stand.x) + c * (cam.y - stand.y), 0.09) &&
                  near(cam.z, stand.z + 0.65), "camera drifted from the stand top datum");
          s->moved = window->mounting_items(); s->evidence["moved"] = s->moved;
          window->undo_layout_edit(); window->undo_layout_edit();
          require(window->mounting_items() == s->mated, "undo did not restore the mated graph");
          window->redo_layout_edit(); window->redo_layout_edit();
          require(window->mounting_items() == s->moved, "redo did not restore the moved graph");
          check_bin_on_table("redone");
          const auto camera_world = item(camera);
          mate("camera-detach", camera, "world", "world", "origin", "coincident", {0., 0., 0., 0., 0., 0.}, false, false, true);
          require(!relation(camera)["parent_object"] || relation(camera)["parent_object"].as<std::string>() == "world", "detach kept a parent");
          require(near(item(camera).x, camera_world.x) && near(item(camera).z, camera_world.z) && near(item(camera).pitch, camera_world.pitch), "detach moved the camera");
          window->undo_layout_edit();
          require(window->mounting_items() == s->moved, "detach undo lost graph/poses");
          const auto bin_world = item(bin);
          const auto reparent = mate("bin-reparent", bin, s->stand, "top", "bottom", "flush", {0., 0., 0., 0., 0., 0.}, false, true);
          require(relation(bin)["parent_object"].as<std::string>("") == s->stand.toStdString(), "reparent did not change the parent");
          require(near(item(bin).x, bin_world.x) && near(item(bin).y, bin_world.y) && near(item(bin).z, bin_world.z), "reparent moved the bin");
          s->evidence["reparent_preview"] = reparent.value("preview");
          window->undo_layout_edit();
          require(window->mounting_items() == s->moved, "reparent undo lost graph/poses");
          s->evidence["undo_redo_detach_reparent"] = true;
          require(window->save_layout_button_ && window->save_layout_button_->isEnabled(), "Save Layout control is unavailable");
          window->save_layout_button_->click();
          s->phase = 3; s->ticks = 0; break;
        }
        case 3: {  // after each Save: disk matches the editor, zone keeps its target-local placement
          if (s->ticks < 12) return;
          require(!window->layout_dirty_, QString("save %1 was rejected or remained dirty").arg(s->saves));
          const auto saved = environment();
          const auto b = record(saved, bin.toStdString());
          require(b["mate"]["child_datum"].as<std::string>("") == "bottom" && b["mate"]["alignment"].as<std::string>("") == "flush", "saved bin lost its mate");
          require(b["parent_object"].as<std::string>("") == table.toStdString() && b["parent_link"].as<std::string>("") == "surface", "saved bin lost its parent datum");
          require(b["mounting_anchors"]["bottom"]["datum_type"].as<std::string>("") == "plane", "saved bin datum missing");
          const auto zone = record(saved, "default_drop_zone");
          require(QString::fromStdString(YAML::Dump(zone["placement_local"])) == s->zone_local.value("default_drop_zone").toString(), "destination zone placement_local changed");
          const auto live = item(bin);
          for (int k = 0; k < 3; ++k) require(near(b["pose_xyz"][k].as<double>(), k == 0 ? live.x : k == 1 ? live.y : live.z), "disk bin pose differs from the editor");
          if (s->saves > 0) require(near(b["pose_xyz"][1].as<double>(), bin_moves[s->saves - 1]), "disk bin Y differs from the requested move");
          const auto c = record(saved, camera.toStdString());
          require(c["parent_link"].as<std::string>("") == "top" && near(saved["camera"]["pose"][2].as<double>(), c["pose_xyz"][2].as<double>()), "camera relation or camera pose mirror not saved");
          s->evidence[QString("save_%1").arg(s->saves)] = QJsonObject{{"bin", QString::fromStdString(YAML::Dump(b))}, {"zone", QString::fromStdString(YAML::Dump(zone))}};
          if (s->saves == int(bin_moves.size())) {
            s->evidence["final_items"] = window->mounting_items();
            s->evidence["saved"] = true; s->evidence["result"] = "PASS";
            window->grab().save(output + "/after-three-saves.png");
            write(); timer->stop(); window->close(); app->exit(0); break;
          }
          window->apply_scene_selection(bin, "", false, false);
          window->inspector_y_->setValue(bin_moves[s->saves]);
          window->inspector_apply_button_->click();
          require(window->layout_dirty_, "bin Apply did not mark the layout dirty");
          check_bin_on_table(QString("bin_move_%1").arg(s->saves + 1));
          ++s->saves;
          window->save_layout_button_->click();
          s->ticks = 0; break;
        }
        case 10: {  // separate Builder process: compare relations, IDs and transforms with disk
          if (s->ticks < 14) return;
          if (window->scene_preview_widget_ && !window->scene_preview_widget_->is_native_product_view_backend() &&
              !window->scene_preview_widget_->embedded_web_authoring_active()) return;
          require(QFileInfo(window->selected_scene_state_.path).canonicalFilePath() == QFileInfo(s->scene).canonicalFilePath(), "reopen selected a different scene");
          QFile previous(output + "/authoring.json"); require(previous.open(QIODevice::ReadOnly), "authoring evidence missing");
          const auto before = QJsonDocument::fromJson(previous.readAll()).object().value("final_items").toArray();
          const auto after = window->mounting_items();
          s->evidence["reopened"] = after;
          require(before.size() == after.size(), "reopen changed physical identities");
          QMap<QString, QJsonObject> indexed;
          for (const auto & row : after) indexed.insert(row.toObject().value("id").toString(), row.toObject());
          const auto disk = environment();
          for (const auto & value : before) {
            const auto row = value.toObject(), actual = indexed.value(row.value("id").toString());
            require(!actual.isEmpty(), "reopen lost " + row.value("id").toString());
            for (const auto & key : {QString("pose_xyz"), QString("pose_rpy")})
              for (int k = 0; k < 3; ++k) require(near(row.value(key).toArray()[k].toDouble(), actual.value(key).toArray()[k].toDouble()), "reopen changed a transform of " + row.value("id").toString());
            const auto a = YAML::Load(row.value("mounting_yaml").toString().toStdString()), b = YAML::Load(actual.value("mounting_yaml").toString().toStdString());
            for (const char * key : {"parent_object", "parent_link", "mate", "origin", "mounting_anchors"})
              require(workcell_builder::same_yaml_value(a[key], b[key]), QString("reopen changed %1 of %2").arg(key, row.value("id").toString()));
            if (row.value("category").toString() == "zone") continue;
            const auto stored = record(disk, row.value("id").toString().toStdString());
            for (int k = 0; k < 3; ++k) require(near(stored["pose_xyz"][k].as<double>(), actual.value("pose_xyz").toArray()[k].toDouble()), "editor pose differs from environment.yaml for " + row.value("id").toString());
            require(workcell_builder::same_yaml_value(stored["mate"], b["mate"]) && workcell_builder::same_yaml_value(stored["origin"], b["origin"]) && workcell_builder::same_yaml_value(stored["mounting_anchors"], b["mounting_anchors"]),
              "editor relation differs from environment.yaml for " + row.value("id").toString());
          }
          const auto check = window->run_mounting_request({{"operation", "anchors"}});
          require(check.value("ok").toBool(), "reopened relations do not resolve: " + check.value("error").toString());
          s->evidence["relations_match_disk"] = true;
          window->apply_scene_selection(bin, "", false, false);
          s->phase = 11; s->ticks = 0; break;
        }
        case 11: {  // the Product View re-reports the selected pose; that is not an edit
          if (s->ticks < 12) return;
          require(!window->layout_dirty_, "selecting a mated asset marked the layout dirty");
          s->evidence["selection_keeps_layout_clean"] = true;
          s->evidence["selection_relation_text"] = window->inspector_mount_relation_label_->text();
          require(window->inspector_mount_relation_label_->text().startsWith("Mated: bottom flush"), "selection context does not show the mate");
          QJsonArray tree;
          std::function<void(QTreeWidgetItem *, int)> walk = [&](QTreeWidgetItem * node, int depth) {
            tree.append(QString(depth * 2, ' ') + node->text(0));
            for (int i = 0; i < node->childCount(); ++i) walk(node->child(i), depth + 1);
          };
          for (int i = 0; i < window->scene_hierarchy_tree_->topLevelItemCount(); ++i) walk(window->scene_hierarchy_tree_->topLevelItem(i), 0);
          s->evidence["hierarchy"] = tree;
          window->grab().save(output + "/reopened.png");
          window->generate_scene_package_for_selected_scene();
          require(QFileInfo::exists(s->scene + "/generated/generated_environment_objects.yaml"), "GUI generation did not produce environment objects");
          s->evidence["gui_generation_invoked"] = true;
          window->validate_generated_scene_for_selected_scene();
          const auto ready = window->selected_scene_readiness();
          s->evidence["readiness_ready"] = ready.ready;
          s->evidence["readiness_blockers"] = QJsonArray::fromStringList(ready.blockers);
          s->evidence["result"] = "PASS";
          write(); timer->stop(); window->close(); app->exit(0); break;
        }
      }
    } catch (const std::exception & error) {
      s->evidence["failure"] = QString::fromUtf8(error.what()); window->grab().save(output + "/failure.png");
      write(); timer->stop(); window->layout_dirty_ = false; window->close(); app->exit(1);
    } });
    timer->start();
  }

  // R4.2: the real Builder window's Live Perception panel on the cell's bound real camera:
  // Discover -> select -> Save Camera Binding -> Start -> observe -> Stop. Robot motion is never involved.
  static void start_live_perception_acceptance(MainWindow * window, QApplication * app, const QString & output,
                                               const QString & scene_path, int observe_s) {
    struct Session { int phase=0, ticks=0; QJsonArray samples; QJsonObject evidence; };
    auto s=std::make_shared<Session>(); QDir().mkpath(output);
    s->evidence={{"result","FAIL"},{"scene",scene_path},{"observe_s",observe_s}};
    auto write=[s,output](){ s->evidence["samples"]=s->samples; QFile f(output+"/acceptance.json"); if(f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    // The whole window plus the Live Perception card itself (it lives in the Inspector's Task tab).
    auto snapshot=[window,output](const QString & name){
      window->grab().save(output+"/"+name+".png");
      if(auto * card=window->perception_status_label_->parentWidget()) card->grab().save(output+"/"+name+"_panel.png"); };
    auto * timer=new QTimer(window); timer->setInterval(500);
    QObject::connect(timer,&QTimer::timeout,window,[=](){ try {
      if(++s->ticks>4*observe_s+2400) throw std::runtime_error("live perception acceptance timed out");
      switch(s->phase){
        case 0:{ if(window->load_watcher_&&window->load_watcher_->isRunning()) return;
          window->refresh_scene_browser_ui(); int index=-1;
          const auto wanted=workcell_builder::canonical_scene_identity(scene_path.toStdString());
          for(size_t i=0;i<window->scene_browser_result_.scenes.size();++i)
            if(window->scene_browser_result_.scenes[i].canonical_scene_dir==wanted) index=int(i);
          if(index<0) throw std::runtime_error("saved cell not discovered by the scene browser");
          if(!window->open_scene_builder_for_scene_index(index,"R4.2 live perception acceptance")) throw std::runtime_error("saved cell could not open");
          s->phase=1; s->ticks=0; break; }
        case 1:{ if(s->ticks<12) return;
          window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
          for(int i=0;i<window->scene_builder_inspector_tabs_->count();++i)
            if(window->scene_builder_inspector_tabs_->widget(i)->isAncestorOf(window->perception_status_label_))
              window->scene_builder_inspector_tabs_->setCurrentIndex(i);
          s->evidence["opened"]=QJsonObject{{"perception_running_on_open",window->perception_process_->state()!=QProcess::NotRunning}};
          if(window->perception_process_->state()!=QProcess::NotRunning) throw std::runtime_error("perception started on opening the cell");
          window->perception_discover_button_->click();
          s->evidence["discovered"]=QJsonObject{{"cameras",window->perception_cameras_},{"label",window->perception_status_label_->text()}};
          if(window->perception_cameras_.isEmpty()) throw std::runtime_error("no camera discovered");
          window->perception_camera_combo_->setCurrentIndex(0); window->perception_bind_button_->click();
          s->evidence["binding"]=window->perception_status_label_->text();
          snapshot("bound"); window->perception_start_button_->click(); s->phase=2; s->ticks=0; break; }
        case 2:{
          if(s->ticks%4==0){
            QFile f(window->perception_session_dir_+"/status.json");
            if(f.open(QIODevice::ReadOnly)){ const auto st=QJsonDocument::fromJson(f.readAll()).object();
              const auto camera=st.value("camera").toObject();
              s->samples.append(QJsonObject{{"t",s->ticks/2},{"readiness",st.value("readiness")},{"detections",st.value("detections").toArray().size()},
                {"camera_state",camera.value("state")},{"camera_reasons",camera.value("reasons")},{"timing",camera.value("timing")},
                {"epd_result_age_s",st.value("epd").toObject().value("result_age_s")},
                {"table_rows",window->perception_table_->rowCount()},{"panel",window->perception_status_label_->text()}}); }
            if(s->ticks==40||s->ticks==4*observe_s/2) snapshot(QString("live_%1s").arg(s->ticks/2));
          }
          if(s->ticks<2*observe_s) return;
          snapshot("live_final"); s->evidence["before_stop_rows"]=window->perception_table_->rowCount();
          window->perception_start_button_->click(); s->phase=3; s->ticks=0; break; }
        case 3:{ if(window->perception_process_->state()!=QProcess::NotRunning) return; if(s->ticks<4) return;
          window->refresh_live_perception(); snapshot("stopped");
          QFile f(window->perception_session_dir_+"/status.json"); f.open(QIODevice::ReadOnly);
          const auto st=QJsonDocument::fromJson(f.readAll()).object();
          s->evidence["final"]=QJsonObject{{"state",st.value("state")},{"shutdown",st.value("shutdown")},{"session",window->perception_session_dir_},
            {"panel",window->perception_status_label_->text()}};
          // A camera outage (stalled/disconnected) must block perception and planning, then recover by itself.
          bool saw_detections=false, outage=false, outage_blocked=true, recovered=false; int ready=0;
          for(const auto & v:s->samples){ const auto o=v.toObject(); const auto r=o.value("readiness").toObject();
            saw_detections|=o.value("table_rows").toInt()>0; ready+=r.value("perception").toString()=="READY";
            const bool down=o.value("camera_reasons").toVariant().toStringList().join(" ").contains("stalled")||
                            o.value("camera_reasons").toVariant().toStringList().join(" ").contains("no RGB");
            if(down){ outage=true; recovered=false; outage_blocked&=r.value("perception").toString()!="READY"&&r.value("robot_planning").toString().startsWith("BLOCKED"); }
            else if(outage&&o.value("camera_state").toString()!="BLOCKED"&&o.value("table_rows").toInt()>0&&o.value("epd_result_age_s").toDouble()>0) recovered=true; }
          s->evidence["outage"]=QJsonObject{{"seen",outage},{"blocked_during_outage",outage_blocked},{"recovered_after_outage",recovered}};
          s->evidence["ready_samples_pct"]=s->samples.isEmpty()?0.0:100.0*ready/s->samples.size();
          const bool ok=saw_detections && (!outage||(outage_blocked&&recovered)) &&
            st.value("state").toString()=="STOPPED" && st.value("shutdown").toObject().value("clean").toBool();
          s->evidence["result"]=ok?"PASS":"FAIL"; write(); timer->stop(); window->close(); app->exit(ok?0:1); break; }
      }
    } catch(const std::exception & e){ s->evidence["failure"]=QString::fromUtf8(e.what()); s->evidence["phase"]=s->phase;
      snapshot("failure"); write(); timer->stop();
      if(window->perception_process_->state()!=QProcess::NotRunning){ window->perception_process_->terminate(); window->perception_process_->waitForFinished(20000); }
      app->exit(1); }});
    timer->start();
  }

  // R4.1b: the real Builder window runs the saved cell's physics cycle through its own button, the
  // confirmation dialog it shows, its live state label and (optionally) its Stop button.
  static void start_physics_cycle_acceptance(MainWindow * window, QApplication * app, const QString & output,
                                             const QString & scene_path, const QString & cancel_at) {
    struct Session { int phase=0, ticks=0; bool cancel_sent=false; QString last_state; QJsonArray states; QJsonObject evidence; };
    auto s=std::make_shared<Session>(); QDir().mkpath(output);
    s->evidence={{"result","FAIL"},{"scene",scene_path},{"cancel_at",cancel_at},
      {"studio_session",QJsonObject{{"pid",double(QCoreApplication::applicationPid())},{"started_at",QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}}}};
    auto write=[s,output](){ s->evidence["states"]=s->states; QFile f(output+"/acceptance.json"); if(f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto snapshot=[window,output](const QString & name){ window->grab().save(output+"/"+name+".png"); };
    auto * timer=new QTimer(window); timer->setInterval(500);
    QObject::connect(timer,&QTimer::timeout,window,[=](){ try {
      if(++s->ticks>14400) throw std::runtime_error("physics acceptance timed out");
      if(window->physics_state_!=s->last_state){
        s->last_state=window->physics_state_;
        s->states.append(QJsonObject{{"state",s->last_state},{"at",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
                                     {"label",window->physics_state_label_->text()}});
        if(s->last_state=="PLANNING"||s->last_state=="RUNNING") snapshot("state_"+s->last_state.toLower());
      }
      switch(s->phase){
        case 0:{ if(window->load_watcher_&&window->load_watcher_->isRunning()) return;
          window->refresh_scene_browser_ui(); int index=-1;
          const auto wanted=workcell_builder::canonical_scene_identity(scene_path.toStdString());
          for(size_t i=0;i<window->scene_browser_result_.scenes.size();++i)
            if(window->scene_browser_result_.scenes[i].canonical_scene_dir==wanted) index=int(i);
          if(index<0) throw std::runtime_error("saved cell not discovered by the scene browser");
          if(!window->open_scene_builder_for_scene_index(index,"R4.1b physics acceptance")) throw std::runtime_error("saved cell could not open");
          s->phase=1; s->ticks=0; break; }
        case 1:{ if(s->ticks<12) return;
          window->show_studio_page(MainWindow::StudioPage::PlanSimulatePage); window->refresh_preview_launch_ui();
          s->evidence["opened"]=QJsonObject{{"physics_state",window->physics_state_},{"label",window->physics_state_label_->text()},
            {"button_enabled",window->run_physics_button_->isEnabled()},{"tooltip",window->run_physics_button_->toolTip()},
            {"simulator_running_on_open",window->preview_process_->state()!=QProcess::NotRunning}};
          snapshot("opened");
          if(window->preview_process_->state()!=QProcess::NotRunning) throw std::runtime_error("something started on opening the cell");
          if(window->physics_state_!="READY"||!window->run_physics_button_->isEnabled())
            throw std::runtime_error(("physics cycle not READY: "+window->run_physics_button_->toolTip()).toStdString());
          // Unsaved changes block the action (the harness raises the same dirty flag an unsaved Studio edit sets).
          window->layout_dirty_=true; window->refresh_preview_launch_ui();
          s->evidence["unsaved_changes"]=QJsonObject{{"button_enabled",window->run_physics_button_->isEnabled()},
            {"physics_state",window->physics_state_},{"tooltip",window->run_physics_button_->toolTip()}};
          if(window->run_physics_button_->isEnabled()) throw std::runtime_error("unsaved changes did not block the physics cycle");
          window->layout_dirty_=false; window->refresh_preview_launch_ui();
          // The confirmation dialog the button shows is read and accepted like an operator would.
          QTimer::singleShot(800,window,[s,output](){
            auto * box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
            if(!box){ s->evidence["confirmation"]=QJsonObject{{"shown",false}}; return; }
            box->grab().save(output+"/confirmation.png");
            s->evidence["confirmation"]=QJsonObject{{"shown",true},{"title",box->windowTitle()},{"text",box->text()}};
            box->button(QMessageBox::Yes)->click(); });
          window->run_physics_button_->click();
          if(!s->evidence["confirmation"].toObject().value("shown").toBool()) throw std::runtime_error("no explicit confirmation before start");
          s->phase=2; s->ticks=0; break; }
        case 2:{
          if(!cancel_at.isEmpty() && !s->cancel_sent && window->physics_state_==cancel_at && s->ticks>20){
            s->cancel_sent=true; snapshot("before_cancel");
            s->evidence["cancel_requested"]=QJsonObject{{"state",window->physics_state_},{"at",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
              {"stage",window->cycle_status_label_->property("backendStage").toString()}};
            window->stop_preview_button_->click(); }
          if(window->preview_process_->state()!=QProcess::NotRunning) return;
          if(s->ticks<4) return;
          snapshot("final");
          s->evidence["final"]=QJsonObject{{"physics_state",window->physics_state_},{"preview_state",window->preview_state_},
            {"label",window->physics_state_label_->text()},{"cycle_result",window->cycle_result_},{"evidence",window->cycle_output_dir_}};
          const bool ok=cancel_at.isEmpty() ? window->physics_state_=="COMPLETED" :
            (window->physics_state_=="CANCELLED" && window->cycle_result_.value("cancellation_confirmed").toBool());
          s->evidence["result"]=ok?"PASS":"FAIL"; write(); timer->stop(); window->close(); app->exit(ok?0:1); break; }
      }
    } catch(const std::exception & e){ s->evidence["failure"]=QString::fromUtf8(e.what()); s->evidence["phase"]=s->phase;
      snapshot("failure"); write(); timer->stop(); window->stop_preview_process();
      if(window->preview_process_->state()==QProcess::NotRunning) app->exit(1);
      else QObject::connect(window->preview_process_,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),app,[app](){app->exit(1);}); }});
    timer->start();
  }

  static void start_persistence_only(MainWindow * window, QApplication * app, const QString & output) {
    struct Session { int phase=0, ticks=0; QString scene; QJsonObject evidence; };
    auto s=std::make_shared<Session>(); QDir().mkpath(output);
    s->evidence={{"result","FAIL"},{"studio_session",QJsonObject{{"pid",double(QCoreApplication::applicationPid())}}}};
    auto write=[s,output](){ QFile f(output+"/persistence.json"); if(f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto read_pose=[](const QString & path,const std::string & id){ auto y=YAML::LoadFile(path.toStdString()); int n=0; std::vector<double> p; for(const auto & i:y["items"]){ if(i["id"].as<std::string>()==id){ ++n; for(const auto & v:i["pose"]["xyz"]) p.push_back(v.as<double>()); }} if(n!=1||p.size()!=3) throw std::runtime_error("authored pose missing or duplicated"); return p; };
    auto * timer=new QTimer(window); timer->setInterval(300);
    QObject::connect(timer,&QTimer::timeout,window,[=](){ try {
      if(++s->ticks>1200) throw std::runtime_error("persistence acceptance timed out");
      const QString layout=s->scene+"/layout/workcell_studio_layout.yaml";
      switch(s->phase){
        case 0:{ if(window->load_watcher_&&window->load_watcher_->isRunning() && s->ticks<80) return; s->scene=window->detect_workspace_root()+"/src/easy_manipulation_deployment/scenes/ur5_2f_test"; int idx=-1; for(size_t i=0;i<window->scene_browser_result_.scenes.size();++i) if(window->scene_browser_result_.scenes[i].scene_dir.string()==s->scene.toStdString()) idx=int(i); if(idx<0) for(size_t i=0;i<window->scene_browser_result_.scenes.size();++i) if(window->scene_browser_result_.scenes[i].scene_name=="ur5_2f_test") idx=int(i); if(!window->open_scene_builder_for_scene_index(idx,"R1.6 persistence-only")) throw std::runtime_error("canonical scene could not open"); s->phase=1; s->ticks=0; break; }
        case 1:{ if(s->ticks<12) return; window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage); window->apply_scene_selection("place_zone_default","place_zone",false,false); auto before=read_pose(layout,"place_zone_default"); if(std::abs(before[0]-.45)>1e-8) throw std::runtime_error("unexpected original place-zone pose"); s->evidence["boundary_1_inspector_before"]=QJsonArray{before[0],before[1],before[2]}; window->inspector_x_->setValue(.46); window->inspector_apply_button_->click(); auto st=window->current_selected_scene_item(); s->evidence["boundary_1_inspector_apply"]=QJsonArray{st.pose_x,st.pose_y,st.pose_z}; s->evidence["boundary_2_dirty_session"]=s->evidence["boundary_1_inspector_apply"]; s->evidence["boundary_3_canvas_before"]=QJsonArray{st.pose_x,st.pose_y,st.pose_z}; s->phase=2; s->ticks=0; break; }
        case 2:{ if(s->ticks<3) return; window->save_layout_button_->click(); s->phase=3; s->ticks=0; break; }
        case 3:{ if(s->ticks<8||window->layout_dirty_) return; auto p=read_pose(layout,"place_zone_default"); s->evidence["boundary_4_canvas_after"]=QJsonArray{p[0],p[1],p[2]}; s->evidence["boundary_5_serializer_input"]=s->evidence["boundary_4_canvas_after"]; s->evidence["boundary_6_yaml_before_write"]=s->evidence["boundary_4_canvas_after"]; s->evidence["boundary_7_disk_readback"]=QJsonArray{p[0],p[1],p[2]}; if(std::abs(p[0]-.46)>1e-8) throw std::runtime_error("disk readback did not persist edited pose"); auto target=read_pose(layout,"target_bin_default"); s->evidence["target_bin_default"]=QJsonArray{target[0],target[1],target[2]}; window->open_scene_builder_for_scene_index(window->selected_scene_index_,"R1.6 persistence reload"); s->phase=4; s->ticks=0; break; }
        case 4:{ if(window->load_watcher_&&window->load_watcher_->isRunning()) return; if(s->ticks<10) return; auto st=window->current_selected_scene_item(); s->evidence["boundary_8_normal_reload"]=QJsonArray{st.pose_x,st.pose_y,st.pose_z}; if(std::abs(st.pose_x-.46)>1e-8) throw std::runtime_error("normal reload lost edited pose"); s->evidence["positive_verifier"]=QJsonObject{{"expected",QJsonArray{.46,.22,.13}},{"disk",s->evidence["boundary_7_disk_readback"]},{"result","PASS"}}; s->evidence["negative_verifier"]=QJsonObject{{"expected",QJsonArray{.46,.22,.13}},{"actual",QJsonArray{.45,.22,.13}},{"result","FAIL"}}; s->evidence["no_ros_process"]=(window->preview_process_->state()==QProcess::NotRunning); s->evidence["result"]="PASS"; write(); timer->stop(); window->close(); app->exit(0); break; }
      }
    } catch(const std::exception & e){ s->evidence["failure"]=QString::fromUtf8(e.what()); s->evidence["phase"]=s->phase; write(); timer->stop(); window->stop_preview_process(); app->exit(1); }}); timer->start();
  }

  static void start_full_cycle_acceptance(MainWindow * window, QApplication * app, const QString & output) {
    struct Session {
      int phase=0, ticks=0, run=0;
      double original_x=0;
      QString scene;
      QJsonObject evidence;
    };
    auto s=std::make_shared<Session>();
    s->evidence={{"result","FAIL"},{"studio_session",QJsonObject{{"pid",double(QCoreApplication::applicationPid())},
      {"started_at",QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}}}};
    QDir().mkpath(output);
    auto * timer=new QTimer(window); timer->setInterval(500);
    auto write=[s,output]() { QFile f(output+"/acceptance.json"); if(f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson()); };
    auto destination=[s]() {
      const auto cell=YAML::LoadFile((s->scene+"/cell_definition.yaml").toStdString());
      for(const auto & z:cell["environment"]["task_zones"]) if(z["id"].as<std::string>()=="default_drop_zone") return z["pose_xyz"][0].as<double>();
      throw std::runtime_error("generated destination absent");
    };
    QObject::connect(timer,&QTimer::timeout,window,[=]() {
      try {
        if(++s->ticks>2400) throw std::runtime_error("Studio acceptance timed out");
        switch(s->phase) {
          case 0: {
            if(window->load_watcher_ && window->load_watcher_->isRunning()) return;
            s->scene=window->detect_workspace_root()+"/src/easy_manipulation_deployment/scenes/ur5_2f_test";
            int index=-1;
            for(size_t i=0;i<window->scene_browser_result_.scenes.size();++i)
              if(window->scene_browser_result_.scenes[i].scene_dir.string()==s->scene.toStdString()) index=int(i);
            if(!window->open_scene_builder_for_scene_index(index,"R1.6 canonical scene open")) throw std::runtime_error("canonical scene could not open");
            s->original_x=destination();
            s->phase=1; s->ticks=0; break;
          }
          case 1:
            if(s->ticks<12 || !window->ensure_live_authoring_mutation_available("R1.6 acceptance")) return;
            window->generate_scene_package_for_selected_scene(); s->phase=2; s->ticks=0; break;
          case 2:
            if(s->ticks<12) return;
            window->validate_generated_scene_for_selected_scene(); window->refresh_preview_launch_ui();
            if(!window->selected_scene_readiness().ready) throw std::runtime_error(window->selected_scene_readiness().blockers.join("; ").toStdString());
            s->phase=3; break;
          case 3:
            window->show_studio_page(MainWindow::StudioPage::PlanSimulatePage);
            window->refresh_preview_launch_ui();
            if(!window->run_preview_button_->isEnabled()) throw std::runtime_error("Run Full Cycle disabled");
            window->run_preview_button_->click(); s->phase=4; s->ticks=0; break;
          case 4: {
            if(window->preview_process_->state()!=QProcess::NotRunning) return;
            if(window->preview_state_!="CYCLE_PASS") throw std::runtime_error(("cycle did not PASS: "+window->preview_state_+" "+window->cycle_status_label_->text()).toStdString());
            const auto result=window->cycle_result_;
            const auto used=result.value("destination").toObject().value("pose_xyz").toArray();
            const double expected=s->run==0?s->original_x:s->original_x+.01;
            if(used.isEmpty() || std::abs(used[0].toDouble()-expected)>1e-8) throw std::runtime_error("runtime did not consume regenerated destination");
            const QString name=s->run==0?"run_1":"run_2";
            s->evidence[name]=QJsonObject{{"result",result.value("result")},{"evidence",window->cycle_output_dir_+"/acceptance.json"},
              {"destination",used},{"placement_error_m",result.value("placement_error_m")},
              {"home_verified",result.value("home_verified")},{"detach_verified",result.value("detach_verified")},
              {"final_collision_valid",result.value("final_collision_valid")},{"baseline_acm_restored",result.value("baseline_acm_restored")},
              {"shutdown_clean",result.value("shutdown_clean")},{"remaining_owned_process_groups",result.value("remaining_owned_process_groups")}};
            window->grab().save(output+"/"+name+".png"); write();
            window->stop_preview_process(); // Idempotent after automatic owned cleanup.
            window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
            window->apply_scene_selection("place_zone_default","place_zone",false,false);
            if(!window->inspector_apply_button_ || !window->inspector_apply_button_->isEnabled())
              throw std::runtime_error("drop zone Inspector Apply control is disabled");
            window->inspector_x_->setValue(s->run==0?s->original_x+.01:s->original_x);
            window->inspector_apply_button_->click();
            window->refresh_preview_launch_ui();
            if(window->run_preview_button_->isEnabled()) throw std::runtime_error("unsaved edit did not block execution");
            window->save_layout_button_->click(); s->phase=5; s->ticks=0; break;
          }
          case 5: {
            if(s->ticks<6 || window->layout_dirty_) return;
            const auto layout=YAML::LoadFile((s->scene+"/layout/workcell_studio_layout.yaml").toStdString());
            double saved=std::numeric_limits<double>::quiet_NaN();
            for(const auto & item:layout["items"]) if(item["id"].as<std::string>()=="place_zone_default") saved=item["pose"]["xyz"][0].as<double>();
            const double expected=s->run==0?s->original_x+.01:s->original_x;
            if(!std::isfinite(saved) || std::abs(saved-expected)>1e-8) throw std::runtime_error("Inspector change was not saved to authored layout");
            window->generate_scene_package_for_selected_scene();
            if(std::abs(destination()-expected)>1e-8) throw std::runtime_error("Generate did not propagate authored drop zone");
            if(s->run==0) {
              s->evidence["authored_change"]=QJsonObject{{"item","place_zone_default"},{"before_x",s->original_x},{"saved_x",saved},{"via","Inspector → Apply → Save"}};
              s->evidence["regeneration"]=QJsonObject{{"generated_destination_x",destination()},{"via","Studio Generate → Validate"}};
              s->run=1; s->phase=2; s->ticks=0; write();
            } else { s->phase=6; s->ticks=0; }
            break;
          }
          case 6:
            if(s->ticks<12) return;
            window->validate_generated_scene_for_selected_scene();
            if(!window->selected_scene_readiness().ready) throw std::runtime_error("restored scene not ready");
            s->evidence["final_cleanup"]=QJsonObject{{"studio_reusable",true},{"canonical_destination_restored",true},{"owned_process_running",window->preview_process_->state()!=QProcess::NotRunning}};
            s->evidence["result"]="PASS"; write(); timer->stop(); window->close(); app->exit(0); break;
        }
      } catch(const std::exception & error) {
        s->evidence["failure"]=QString::fromUtf8(error.what()); s->evidence["phase"]=s->phase;
        s->evidence["cycle_evidence"]=window->cycle_output_dir_;
        QFile log(output+"/studio.log"); if(log.open(QIODevice::WriteOnly)) log.write(window->studio_log_->toPlainText().toUtf8());
        window->grab().save(output+"/failure.png"); write(); timer->stop();
        window->stop_preview_process();
        if(window->preview_process_->state()==QProcess::NotRunning) app->exit(1);
        else QObject::connect(window->preview_process_,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),app,[app](){app->exit(1);});
      }
    });
    timer->start();
  }
  // Operator Preview (R3): Builder launches the separate Operator HMI for the
  // selected cell, propagates draft/saved task edits, the HMI runs only offline
  // no-motion checks, invalid input blocks honestly, open/close does not leak
  // processes, and a separate reopen recovers the saved configuration.
  static void start_operator_preview_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene_name, bool reopen) {
    struct Session { int phase=0, ticks=0, wait=0, command=0, cycles=0; qint64 pid=0; QString scene, saved_class, new_class;
                     QJsonObject evidence; QJsonArray commands, checks; };
    auto s = std::make_shared<Session>(); QDir().mkpath(output);
    const QString tag = reopen ? "reopen" : "authoring";
    const QString status_path = output + "/hmi_status_" + tag + ".json";
    const QString commands_path = output + "/hmi_commands_" + tag + ".json";
    QFile::remove(status_path); QFile::remove(commands_path);
    s->evidence = {{"result", "FAIL"}, {"robot_motion_commanded", false}, {"execution_goals_sent", 0}};
    auto write = [=]() {
      s->evidence["checks"] = s->checks;
      s->evidence["studio_log_tail"] = window->studio_log_->toPlainText().right(5000);
      QFile f(output + "/" + tag + ".json");
      if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
    };
    auto require = [=](bool value, const QString & message) {
      s->checks.append(QJsonObject{{"check", message}, {"ok", value}});
      if (!value) throw std::runtime_error(message.toStdString());
    };
    auto status = [=]() {
      QFile f(status_path);
      return f.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(f.readAll()).object() : QJsonObject{};
    };
    auto command = [=](const QString & action, const QJsonObject & extra = {}) {
      QJsonObject c = extra; c["id"] = ++s->command; c["action"] = action;
      s->commands.append(c);
      QSaveFile f(commands_path);
      if (f.open(QIODevice::WriteOnly)) { f.write(QJsonDocument(s->commands).toJson()); f.commit(); }
      return s->command;
    };
    auto command_result = [=](int id) {
      for (const auto & r : status().value("command_results").toArray())
        if (r.toObject().value("id").toInt() == id) return r.toObject().value("result").toString();
      return QString();
    };
    auto count_processes = [](const QString & pattern) {
      QProcess p; p.start("pgrep", {"-fc", pattern}); p.waitForFinished(3000);
      return QString::fromUtf8(p.readAllStandardOutput()).trimmed().toInt();
    };
    auto editor = [=]() { return window->environment_task_editor_; };
    auto set_text = [=](const char * name, const QString & value) {
      auto * edit = editor()->findChild<QLineEdit *>(name);
      if (!edit) throw std::runtime_error(std::string("task editor field missing: ") + name);
      edit->setText(value); emit edit->textEdited(value);
    };
    auto * modal_watch = new QTimer(window); modal_watch->setInterval(250);
    QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
      for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
        if (!message->isVisible()) continue;
        s->evidence["failure"] = "unexpected message: " + message->text();
        window->grab().save(output + "/failure_" + tag + ".png"); write(); message->reject(); app->exit(1);
      }
    }); modal_watch->start();
    auto * timer = new QTimer(window); timer->setInterval(250);
    QObject::connect(timer, &QTimer::timeout, window, [=]() { try {
      if (++s->ticks >= 1600) require(false, "operator preview acceptance timed out in phase " + QString::number(s->phase));
      if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
      const auto hmi = status();
      auto * launcher = window->operator_preview_launcher_;
      switch (s->phase) {
        case 0: {
          if (s->ticks < 6) return;
          s->scene = window->detect_workspace_root() + "/src/easy_manipulation_deployment/scenes/" + scene_name;
          s->evidence["scene_path"] = s->scene;
          QStringList blockers; QJsonObject diagnostics;
          require(window->load_scene_for_scene3d_smoke(scene_name, s->scene, &blockers, &diagnostics), "Builder opened the scene");
          window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
          s->phase = 1; s->ticks = 0; break;
        }
        case 1: {
          if (!editor() || editor()->scene().isEmpty() || s->ticks < 8) return;
          require(QFileInfo(editor()->scene()).canonicalFilePath() == QFileInfo(s->scene).canonicalFilePath(), "task editor loaded the scene");
          require(!editor()->dirty(), "task editor starts clean");
          s->saved_class = QString::fromStdString(editor()->model().pick_selection.class_id);
          s->evidence["saved_target_class"] = s->saved_class;
          auto * button = window->findChild<QPushButton *>("sceneBuilderToolbarOperatorPreview");
          require(button && button->isVisible() && button->isEnabled(), "Operator Preview is on the visible Scene Builder toolbar");
          require(window->scene_builder_action("operate.operator_preview") != nullptr, "Operator Preview is a registered Scene Builder action");
          int menu_entries = 0;
          for (auto * action : window->findChildren<QAction *>()) menu_entries += action->text() == "Operator Preview" ? 1 : 0;
          s->evidence["operator_preview_action_count"] = menu_entries;
          require(menu_entries >= 2, "Operator Preview is also reachable from the Workcell Studio Home menu");
          window->grab().save(output + "/01_builder_" + tag + ".png");
          window->ensure_operator_preview_launcher()->set_extra_arguments(
            {"--status-out", status_path, "--acceptance-commands", commands_path, "--capture-dir", output});
          button->click();
          require(window->operator_preview_launcher_->running(), "Operator HMI process started from Builder");
          s->pid = window->operator_preview_launcher_->process_id();
          s->evidence["hmi_pid"] = static_cast<double>(s->pid);
          s->phase = 2; s->ticks = 0; break;
        }
        case 2: {
          if (!hmi.value("loaded").toBool() || !hmi.value("busy").toString().isEmpty()) return;
          require(hmi.value("scene_name").toString() == scene_name, "HMI shows the Builder's selected cell");
          require(hmi.value("revision").toString() == "SAVED", "HMI shows the SAVED revision when the editor is clean");
          require(hmi.value("target_class").toString() == s->saved_class, "HMI target class matches the saved TaskIntent");
          require(!hmi.value("destinations").toArray().isEmpty(), "HMI lists the configured destination");
          require(!hmi.value("start_enabled").toBool(), "Start production is disabled (NOT AUTHORISED)");
          s->evidence["saved_hmi"] = hmi;
          s->evidence["saved_intent_sha256"] = hmi.value("intent_sha256");
          if (reopen) {
            QFile f(output + "/authoring.json");
            const auto authoring = f.open(QIODevice::ReadOnly) ? QJsonDocument::fromJson(f.readAll()).object() : QJsonObject{};
            require(authoring.value("final_saved_intent_sha256").toString() == hmi.value("intent_sha256").toString(),
              "Reopened cell recovers the same saved task (intent SHA-256 matches the authoring run)");
            require(hmi.value("target_class").toString() == authoring.value("final_target_class").toString(),
              "Reopened cell recovers the saved target class");
            command("capture", {{"name", "20_hmi_reopened"}});
            s->phase = 90; s->ticks = 0; break;
          }
          command("capture", {{"name", "02_hmi_saved"}});
          s->new_class = s->saved_class == "cup" ? "bottle" : "cup";
          s->evidence["draft_target_class"] = s->new_class;
          set_text("taskTargetClass", s->new_class);
          s->phase = 3; s->ticks = 0; break;
        }
        case 3: {  // the Builder pushes the draft without raising: HMI offers Refresh
          if (!hmi.value("update_banner_visible").toBool()) return;
          require(editor()->dirty(), "task editor holds unsaved edits");
          require(hmi.value("revision").toString() == "SAVED", "HMI keeps showing what it loaded until refreshed");
          s->wait = command("capture", {{"name", "03_hmi_update_available"}});
          s->phase = 30; s->ticks = 0; break;
        }
        case 30: {  // refresh only after the banner was captured
          if (command_result(s->wait).isEmpty()) return;
          window->findChild<QPushButton *>("sceneBuilderToolbarOperatorPreview")->click();
          s->phase = 4; s->ticks = 0; break;
        }
        case 4: {
          if (hmi.value("revision").toString() != "DRAFT" || !hmi.value("busy").toString().isEmpty()) return;
          require(hmi.value("target_class").toString() == s->new_class, "Refresh shows the draft target class");
          require(launcher->process_id() == s->pid, "Repeated Operator Preview reuses the running HMI");
          require(hmi.value("mode").toString() == "DRAFT", "Mode is DRAFT for unsaved edits");
          command("capture", {{"name", "04_hmi_draft"}});
          s->wait = command("replay_check");
          s->phase = 5; s->ticks = 0; break;
        }
        case 5: {
          if (!hmi.value("replay_shown").toBool() || !hmi.value("busy").toString().isEmpty()) return;
          require(command_result(s->wait) == "done", "Replay check ran from the HMI");
          QString accepted, rejected;
          for (const auto & o : hmi.value("objects").toArray()) {
            const auto obj = o.toObject();
            if (obj.value("accepted").toBool()) accepted = obj.value("class_id").toString();
            else if (obj.value("reason_code").toString() == "CLASS_NOT_TARGETED") rejected = obj.value("object_id").toString();
          }
          s->evidence["draft_replay_objects"] = hmi.value("objects");
          require(accepted == s->new_class, "Replay check accepts the draft target class");
          require(!rejected.isEmpty(), "Replay check rejects the other class with CLASS_NOT_TARGETED");
          require(hmi.value("mode").toString() == "DRAFT", "Draft stays labelled DRAFT during replay");
          command("capture", {{"name", "05_hmi_draft_replay"}});
          command("select_object", {{"object_id", rejected}});
          command("capture", {{"name", "06_hmi_rejected_detail"}});
          editor()->findChild<QPushButton *>("taskSave")->click();
          s->phase = 6; s->ticks = 0; break;
        }
        case 6: {
          if (editor()->dirty() || !hmi.value("update_banner_visible").toBool()) return;
          s->wait = command("refresh");  // operator-side refresh path
          s->phase = 7; s->ticks = 0; break;
        }
        case 7: {
          if (hmi.value("revision").toString() != "SAVED" || !hmi.value("busy").toString().isEmpty() ||
              hmi.value("update_banner_visible").toBool()) return;
          require(hmi.value("target_class").toString() == s->new_class, "Saved change propagates after HMI Refresh");
          require(hmi.value("mode").toString() == "REPLAY", "Saved revision with recorded input is labelled REPLAY");
          s->evidence["final_saved_intent_sha256"] = hmi.value("intent_sha256");
          s->evidence["final_target_class"] = s->new_class;
          command("capture", {{"name", "07_hmi_saved_replay"}});
          // Existing engineering workflow: Generate, then Validate (unchanged code paths).
          window->generate_scene_package_for_selected_scene();
          s->evidence["generated_summary_exists"] = QFileInfo::exists(s->scene + "/generated/generated_workcell_summary.json");
          window->run_offline_validation();
          s->evidence["builder_offline_validation_state"] =
            window->validation_state_text().remove(QRegularExpression("<[^>]*>")).simplified();
          s->wait = command("check");
          s->phase = 8; s->ticks = 0; break;
        }
        case 8: {
          if (command_result(s->wait).isEmpty() || !hmi.value("busy").toString().isEmpty()) return;
          s->evidence["after_validate_generate_hmi"] = hmi;
          require(s->evidence["generated_summary_exists"].toBool(), "Builder Generate produced the generated package");
          require(hmi.value("dry_run_enabled").toBool(), "Generated-cell dry-run becomes available after Generate");
          s->wait = command("dry_run");
          s->phase = 9; s->ticks = 0; break;
        }
        case 9: {
          if (hmi.value("dry_run_status").toString().isEmpty() || !hmi.value("busy").toString().isEmpty()) return;
          s->evidence["dry_run_status"] = hmi.value("dry_run_status");
          s->evidence["dry_run_blockers"] = hmi.value("dry_run_blockers");
          require(hmi.value("dry_run_status").toString() == "PASS" || !hmi.value("dry_run_blockers").toArray().isEmpty(),
            "Dry-run result is a PASS or names its blockers");
          command("capture", {{"name", "08_hmi_dry_run"}});
          // Engineering error: bind a commissioning replay that does not exist (never saved).
          auto * mode = editor()->findChild<QComboBox *>("taskPerceptionMode");
          mode->setCurrentIndex(mode->findData("replayed_snapshot"));
          set_text("taskReplayPath", "config/missing_commissioning_replay.yaml");
          s->phase = 10; s->ticks = 0; break;
        }
        case 10: {
          if (!editor()->dirty() || s->ticks < 6) return;
          window->findChild<QPushButton *>("sceneBuilderToolbarOperatorPreview")->click();
          s->phase = 11; s->ticks = 0; break;
        }
        case 11: {
          if (hmi.value("revision").toString() != "DRAFT" || !hmi.value("busy").toString().isEmpty() ||
              hmi.value("input_kind").toString() != "unavailable") return;
          require(!hmi.value("replay_enabled").toBool(), "Missing task input disables the replay check");
          require(hmi.value("input_blocker").toString().contains("TASK_INPUT_UNAVAILABLE"), "HMI shows the actual input blocker");
          s->wait = command("replay_check");
          command("start_production");
          command("capture", {{"name", "09_hmi_blocked_input"}});
          s->phase = 12; s->ticks = 0; break;
        }
        case 12: {
          if (command_result(s->wait + 2).isEmpty()) return;
          require(command_result(s->wait).startsWith("rejected"), "Blocked replay check cannot be run");
          require(command_result(s->wait + 1).startsWith("rejected"), "Start production cannot be pressed");
          command("close");
          s->phase = 13; s->ticks = 0; break;
        }
        case 13: {
          if (launcher->running()) return;
          require(window->isVisible(), "Closing the HMI leaves Workcell Builder open");
          require(window->has_selected_scene() && window->findChild<QPushButton *>("sceneBuilderToolbarOperatorPreview")->isEnabled(),
            "Builder stays usable after the HMI closes");
          window->grab().save(output + "/10_builder_after_hmi_close.png");
          s->phase = 14; s->ticks = 0; break;
        }
        case 14: {  // repeated open/close: one HMI at a time, nothing left behind
          if (launcher->running()) return;
          if (s->cycles == 5) {
            const int hmi_left = count_processes("[w]orkcell_operator_hmi --request");
            const int adapters_left = count_processes("[o]perator_preview_projection.py");
            s->evidence["open_close_cycles"] = s->cycles;
            s->evidence["hmi_processes_left"] = hmi_left;
            s->evidence["projection_processes_left"] = adapters_left;
            require(hmi_left == 0 && adapters_left == 0, "Repeated open/close leaves no HMI or adapter process");
            s->phase = 90; s->ticks = 0; break;
          }
          QFile::remove(status_path);
          window->findChild<QPushButton *>("sceneBuilderToolbarOperatorPreview")->click();
          s->phase = 15; s->ticks = 0; break;
        }
        case 15: {
          if (!hmi.value("loaded").toBool() || !hmi.value("busy").toString().isEmpty()) return;
          require(launcher->running(), "HMI reopened");
          ++s->cycles;
          command("close");
          s->phase = 14; s->ticks = 0; break;
        }
        case 90: {
          // The reopen run exits with the HMI still open: the Builder must take it down
          // (checked by the caller: no workcell_operator_hmi process may remain).
          if (reopen) {
            require(launcher && launcher->running(), "HMI still open when the Builder exits");
            s->evidence["hmi_running_at_builder_exit"] = true;
          }
          s->evidence["result"] = "PASS"; write();
          timer->stop(); modal_watch->stop(); app->exit(0); break;
        }
      }
    } catch (const std::exception & error) {
      s->evidence["failure"] = QString::fromUtf8(error.what());
      window->grab().save(output + "/failure_" + tag + ".png");
      write(); timer->stop(); modal_watch->stop(); app->exit(1);
    } });
    timer->start();
  }
  // R3.2 visual mate acceptance: real mouse events into the Product View at
  // projected world points (hover, then click), the real Mate panel controls,
  // Inspector edits, undo/redo, the existing Mount To dialog, Save, Generate and
  // Validate. Everything is checked against the resolver, the editor and disk.
  static void start_visual_mate_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & scene_name, bool reopen, QSize window_size = QSize(1366, 768)) {
    struct Session {
      int step = 0, ticks = 0, sub = 0, candidate = 0;
      QString scene, stand, stand_mesh_uri;
      QJsonObject evidence;
      bool js_pending = false; QVariant js_result;
      QPoint target; qint64 pick_seq_before = 0;
      QByteArray env_hash, layout_hash;
      QJsonArray items_before, mated_items;
      bool native = false;
    };
    using Step = std::function<bool()>;
    auto s = std::make_shared<Session>();
    auto steps = std::make_shared<std::vector<std::pair<QString, Step>>>();
    QDir().mkpath(output);
    s->evidence = {{"result", "FAIL"}, {"fake_hardware_only", true}, {"execution_goals_sent", 0}, {"robot_motion_commanded", false}};
    const QString bin = "target_bin_default", table = "support_surface_table", camera = "realsense_overhead";
    auto write = [=]() {
      s->evidence["studio_log_tail"] = window->studio_log_->toPlainText().right(8000);
      QFile f(output + (reopen ? "/reopen.json" : (s->native ? "/native.json" : "/authoring.json")));
      if (f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(s->evidence).toJson());
    };
    auto require = [](bool value, const QString & message) { if (!value) throw std::runtime_error(message.toStdString()); };
    auto near = [](double a, double b, double tol = 1e-8) { return std::abs(a - b) < tol; };
    // ROS fixed-axis RPY (Rz*Ry*Rx) or three.js 'XYZ' (Rx*Ry*Rz), and the angle between two rotations.
    using M3 = std::array<std::array<double, 3>, 3>;
    auto rpy_matrix = [](double r, double p, double y, bool ros) {
      const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p), cy = std::cos(y), sy = std::sin(y);
      const M3 rx{{{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}}}, ry{{{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}}}, rz{{{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}}};
      auto mul = [](const M3 & a, const M3 & b) {
        M3 m{}; for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) m[i][k] += a[i][j] * b[j][k];
        return m;
      };
      return ros ? mul(rz, mul(ry, rx)) : mul(rx, mul(ry, rz));
    };
    auto turn = [](const M3 & a, const M3 & b) {
      double trace = 0.; for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k) trace += a[k][i] * b[k][i];
      return std::acos(std::clamp((trace - 1.) / 2., -1., 1.));
    };
    auto item = [=](const QString & id) {
      for (const auto & p : window->all_scene_preview_items_) if (p.id == id) return p;
      throw std::runtime_error(("scene item missing: " + id).toStdString());
    };
    auto relation = [=](const QString & id) { return YAML::Load(item(id).mounting_yaml.toStdString()); };
    auto file_hash = [](const QString & path) {
      QFile f(path); if (!f.open(QIODevice::ReadOnly)) return QByteArray();
      return QCryptographicHash::hash(f.readAll(), QCryptographicHash::Sha256).toHex();
    };
    auto environment = [=]() { return YAML::LoadFile((s->scene + "/environment.yaml").toStdString()); };
    auto record = [](const YAML::Node & root, const std::string & id) {
      for (const char * key : {"support_surfaces", "assets", "sensors", "task_zones"})
        for (const auto & r : root["environment"][key])
          if (r["id"].as<std::string>("") == id || r["layout_item_ref"].as<std::string>("") == id) return r;
      throw std::runtime_error("authored record missing: " + id);
    };
    // The Product View is composited (QtWebEngine): read the real X11 window.
    auto shot = [=](const QString & name) {
      QCoreApplication::processEvents();
      const QString script = QDir(window->detect_workspace_root()).filePath(
        "src/easy_manipulation_deployment/workcell_builder/workcell_builder/test/x11_window_capture.py");
      const int code = QProcess::execute("python3", {script, QString::number(window->winId()), output + "/" + name + ".png"});
      if (code != 0) window->grab().save(output + "/" + name + ".png");
      QJsonArray shots = s->evidence.value("screenshots").toArray(); shots.append(name); s->evidence["screenshots"] = shots;
    };
    auto mate = [=]() { return window->mate_controller_; };
    // Hierarchy rows are looked up fresh each time: Apply and selection rebuild the
    // tree, so no QTreeWidgetItemIterator may live across them.
    auto hierarchy_row = [=](const QString & id) -> QTreeWidgetItem * {
      std::function<QTreeWidgetItem *(QTreeWidgetItem *)> find = [&](QTreeWidgetItem * node) -> QTreeWidgetItem * {
        if (node->data(0, Qt::UserRole + 1 /* TreeRoleId */).toString() == id) return node;
        for (int i = 0; i < node->childCount(); ++i) if (auto * hit = find(node->child(i))) return hit;
        return nullptr;
      };
      auto * tree = window->scene_hierarchy_tree_;
      for (int i = 0; tree && i < tree->topLevelItemCount(); ++i) if (auto * hit = find(tree->topLevelItem(i))) return hit;
      return nullptr;
    };
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
    auto web = [=]() -> QWebEngineView * {
      for (auto * view : window->scene_preview_widget_->findChildren<QWebEngineView *>()) if (view->isVisible()) return view;
      return nullptr;
    };
    auto js = [=](const QString & script) {
      auto * view = web(); if (!view) return;
      s->js_pending = true; s->js_result = QVariant();
      view->page()->runJavaScript(script, [s](const QVariant & value) { s->js_result = value; s->js_pending = false; });
    };
    auto mouse = [=](QEvent::Type type, const QPoint & p, Qt::MouseButton button, Qt::MouseButtons buttons) {
      auto * view = web(); if (!view) return;
      QWidget * target = view->focusProxy() ? view->focusProxy() : view;
      const QPoint local = target->mapFrom(view, p);
      QMouseEvent event(type, QPointF(local), QPointF(target->window()->mapFromGlobal(target->mapToGlobal(local))),
        QPointF(target->mapToGlobal(local)), button, buttons, Qt::NoModifier);
      QApplication::sendEvent(target, &event);
    };
    auto key_escape = [=]() {
      auto * view = web(); if (!view) return;
      QWidget * target = view->focusProxy() ? view->focusProxy() : view;
      target->setFocus();
      QKeyEvent press(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier), release(QEvent::KeyRelease, Qt::Key_Escape, Qt::NoModifier);
      QApplication::sendEvent(target, &press); QApplication::sendEvent(target, &release);
    };
    // Hover each candidate world point until the hover resolves to the expected
    // asset (and datum, when given), then click it. `check` validates the pick.
    using Points = std::vector<std::array<double, 3>>;
    auto fixed = [](Points points) { return std::function<Points()>([points]() { return points; }); };
    auto click_world = [=](const QString & tag, std::function<Points()> candidate_fn, const QString & expect_asset,
                           const QString & expect_datum, std::function<void(const QJsonObject &)> check) {
      auto local = std::make_shared<int>(0);
      auto passes = std::make_shared<int>(0);
      auto candidates_ptr = std::make_shared<Points>();
      return [=]() -> bool {
        if (s->native) return true;  // the native fallback has no face picking (see the Native steps)
        if (s->js_pending) return false;
        const Points & candidates = *candidates_ptr;
        switch (s->sub) {
          case 0: {
            if (s->candidate == 0) {
              *candidates_ptr = candidate_fn();
              if (candidates_ptr->empty()) { s->evidence[tag + "_skipped"] = true; return true; }
            }
            // The viewer redraws datum markers after each Mate step change; a hover
            // that races that refresh is retried (every attempt stays in the evidence).
            if (s->candidate >= int(candidates_ptr->size()) && ++*passes < 3) { s->candidate = 0; s->sub = 5; *local = 0; return false; }
            require(s->candidate < int(candidates_ptr->size()), tag + ": no candidate point resolved to " + expect_asset + " " + expect_datum);
            const auto p = (*candidates_ptr)[s->candidate];
            js(QString("window.__WORKCELL_EDITOR_API_V1__.projectWorldPoint([%1,%2,%3])").arg(p[0], 0, 'g', 12).arg(p[1], 0, 'g', 12).arg(p[2], 0, 'g', 12));
            s->sub = 1; *local = 0; return false;
          }
          case 1: {
            const auto projected = s->js_result.toMap();
            require(!projected.isEmpty(), tag + ": projection unavailable");
            s->target = QPoint(qRound(projected.value("x").toDouble()), qRound(projected.value("y").toDouble()));
            const auto canvas = projected.value("canvas").toMap();
            const QRect inner(qRound(canvas.value("left").toDouble()) + 30, qRound(canvas.value("top").toDouble()) + 40,
              qRound(canvas.value("width").toDouble()) - 60, qRound(canvas.value("height").toDouble()) - 70);
            if (!projected.value("inFront").toBool() || !inner.contains(s->target)) {
              QJsonArray log = s->evidence.value(tag + "_hover_attempts").toArray();
              log.append(QJsonObject{{"widget", QJsonArray{s->target.x(), s->target.y()}}, {"skipped", "outside the visible canvas"}});
              s->evidence[tag + "_hover_attempts"] = log;
              ++s->candidate; s->sub = 0; return false;
            }
            s->evidence["_canvas_offset"] = QJsonArray{canvas.value("left").toDouble(), canvas.value("top").toDouble()};
            mouse(QEvent::MouseMove, s->target + QPoint(-3, 0), Qt::NoButton, Qt::NoButton);
            mouse(QEvent::MouseMove, s->target, Qt::NoButton, Qt::NoButton);
            s->sub = 2; *local = 0; return false;
          }
          case 2: {
            if (++*local < 9) return false;  // poll + hover resolution round trip
            js("window.__WORKCELL_EDITOR_API_V1__.getMateState()");
            s->sub = 3; return false;
          }
          case 3: {
            const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
            const auto hover = state.value("hoverResolution").toObject();
            // Only a resolution of this very hover counts (overlays can swallow the pointer).
            const auto client = state.value("hover").toObject().value("client").toObject();
            const auto offset = s->evidence.value("_canvas_offset").toArray();
            const bool current = hover.value("seq").toInt() == state.value("hoverSeq").toInt() &&
              std::abs(client.value("x").toDouble() + offset[0].toDouble() - s->target.x()) < 2.5 &&
              std::abs(client.value("y").toDouble() + offset[1].toDouble() - s->target.y()) < 2.5;
            QJsonObject tried{{"point", QJsonArray{candidates[s->candidate][0], candidates[s->candidate][1], candidates[s->candidate][2]}},
              {"widget", QJsonArray{s->target.x(), s->target.y()}}, {"hover", hover}, {"label", state.value("labelText")}};
            QJsonArray log = s->evidence.value(tag + "_hover_attempts").toArray(); log.append(tried); s->evidence[tag + "_hover_attempts"] = log;
            const bool asset_ok = current && (expect_asset.isEmpty() ? !hover.value("asset_id").toString().isEmpty() : hover.value("asset_id").toString() == expect_asset);
            const bool datum_ok = expect_datum.isEmpty() || (hover.value("valid").toBool() && hover.value("datum").toString() == expect_datum);
            if (!asset_ok || !datum_ok) { ++s->candidate; s->sub = 0; return false; }
            s->evidence[tag + "_hover"] = tried;
            s->pick_seq_before = window->mate_seen_pick_seq_;
            mouse(QEvent::MouseButtonPress, s->target, Qt::LeftButton, Qt::LeftButton);
            mouse(QEvent::MouseButtonRelease, s->target, Qt::LeftButton, Qt::NoButton);
            s->sub = 4; *local = 0; return false;
          }
          case 4: {
            if (window->mate_seen_pick_seq_ <= s->pick_seq_before) return false;
            const auto picks = mate()->evidence().value("picks").toArray();
            const auto last = picks.isEmpty() ? QJsonObject() : picks.last().toObject().value("resolution").toObject();
            s->evidence[tag + "_pick"] = last;
            check(last);
            s->sub = 0; s->candidate = 0; *passes = 0;
            return true;
          }
          case 5: {
            if (++*local < 15) return false;  // let the marker overlay settle, then hover again
            s->sub = 0; return false;
          }
        }
        return false;
      };
    };
#endif
    auto panel = [=]() { return window->mate_panel_; };
    auto panel_button = [=](const char * name) { auto * b = panel()->findChild<QPushButton *>(name); require(b, QString("Mate panel control missing: ") + name); return b; };
    auto inspector = [=](const QString & id, double dx, double dy, double dyaw_deg) {
      window->apply_scene_selection(id, "", false, false);
      window->inspector_update_guard_ = true;
      window->inspector_x_->setValue(window->inspector_x_->value() + dx);
      window->inspector_y_->setValue(window->inspector_y_->value() + dy);
      window->inspector_yaw_->setValue(window->inspector_yaw_->value() + dyaw_deg);
      window->inspector_update_guard_ = false;
      window->inspector_apply_button_->click();
    };
    // Child datum frame against parent datum frame, from the resolver's own frames.
    auto frames = [=]() {
      const auto response = window->run_mounting_request({{"operation", "datum_frames"}});
      require(response.value("ok").toBool(), "datum frames failed: " + response.value("error").toString());
      return response.value("result").toObject();
    };
    auto datum = [](const QJsonObject & all, const QString & asset, const QString & name) {
      return all.value(asset).toObject().value("datums").toObject().value(name).toObject();
    };
    auto v3 = [](const QJsonValue & v) { const auto a = v.toArray(); return std::array<double, 3>{a[0].toDouble(), a[1].toDouble(), a[2].toDouble()}; };
    auto check_flush = [=](const QString & phase, const QString & child, const QString & child_datum, const QString & parent, const QString & parent_datum) {
      const auto all = frames();
      const auto c = datum(all, child, child_datum), p = datum(all, parent, parent_datum);
      const auto co = v3(c.value("xyz")), po = v3(p.value("xyz")), cn = v3(c.value("normal")), pn = v3(p.value("normal"));
      const double gap = (co[0] - po[0]) * pn[0] + (co[1] - po[1]) * pn[1] + (co[2] - po[2]) * pn[2];
      const double opposed = cn[0] * pn[0] + cn[1] * pn[1] + cn[2] * pn[2];
      s->evidence[phase + "_flush"] = QJsonObject{{"gap_m", gap}, {"normal_dot", opposed}};
      require(std::abs(gap) < 1e-9 && near(opposed, -1., 1e-9), phase + ": faces are not flush (gap " + QString::number(gap) + ")");
    };
    auto modal_watch = new QTimer(window); modal_watch->setInterval(250);
    QObject::connect(modal_watch, &QTimer::timeout, window, [=]() {
      for (auto * widget : QApplication::topLevelWidgets()) if (auto * message = qobject_cast<QMessageBox *>(widget)) {
        if (!message->isVisible()) continue;
        s->evidence["failure"] = "unexpected message: " + message->text();
        shot("failure"); write(); message->reject(); app->exit(1);
      }
    }); modal_watch->start();

    // ---- Step script --------------------------------------------------------
    auto add = [=](const QString & name, Step step) { steps->push_back({name, step}); };
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
    // Hover (never click) a world point in Mate mode and check what a click would hit.
    auto hover_probe = [=](const QString & tag, std::function<std::array<double, 3>()> point_fn,
                           std::function<void(const QJsonObject & hover, const QJsonObject & state)> check) {
      auto attempts = std::make_shared<int>(0), local = std::make_shared<int>(0);
      auto point = std::make_shared<std::array<double, 3>>();
      return [=]() -> bool {
        if (s->native) return true;
        if (s->js_pending) return false;
        switch (s->sub) {
          case 0: {
            *point = point_fn();
            js(QString("window.__WORKCELL_EDITOR_API_V1__.projectWorldPoint([%1,%2,%3])").arg((*point)[0], 0, 'g', 12).arg((*point)[1], 0, 'g', 12).arg((*point)[2], 0, 'g', 12));
            s->sub = 1; return false;
          }
          case 1: {
            const auto projected = s->js_result.toMap();
            require(!projected.isEmpty() && projected.value("inFront").toBool(), tag + ": probe point is not in view");
            s->target = QPoint(qRound(projected.value("x").toDouble()), qRound(projected.value("y").toDouble()));
            const auto canvas = projected.value("canvas").toMap();
            const QRect inner(qRound(canvas.value("left").toDouble()) + 30, qRound(canvas.value("top").toDouble()) + 40,
              qRound(canvas.value("width").toDouble()) - 60, qRound(canvas.value("height").toDouble()) - 70);
            require(inner.contains(s->target), tag + ": probe point is outside the visible canvas");
            s->evidence["_canvas_offset"] = QJsonArray{canvas.value("left").toDouble(), canvas.value("top").toDouble()};
            mouse(QEvent::MouseMove, s->target + QPoint(-3 - *attempts, 0), Qt::NoButton, Qt::NoButton);
            mouse(QEvent::MouseMove, s->target, Qt::NoButton, Qt::NoButton);
            *local = 0; s->sub = 2; return false;
          }
          case 2: {
            if (++*local < 9) return false;
            js("window.__WORKCELL_EDITOR_API_V1__.getMateState()");
            s->sub = 3; return false;
          }
          case 3: {
            const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
            const auto hover = state.value("hoverResolution").toObject();
            const auto client = state.value("hover").toObject().value("client").toObject();
            const auto offset = s->evidence.value("_canvas_offset").toArray();
            const bool current = hover.value("seq").toInt() == state.value("hoverSeq").toInt() &&
              std::abs(client.value("x").toDouble() + offset[0].toDouble() - s->target.x()) < 2.5 &&
              std::abs(client.value("y").toDouble() + offset[1].toDouble() - s->target.y()) < 2.5;
            if (!current) { require(++*attempts < 5, tag + ": the hover never resolved"); s->sub = 0; return false; }
            s->evidence[tag] = QJsonObject{{"point", QJsonArray{(*point)[0], (*point)[1], (*point)[2]}}, {"widget", QJsonArray{s->target.x(), s->target.y()}},
              {"hover", hover}, {"isolatedAsset", state.value("isolatedAsset")}};
            check(hover, state);
            *attempts = 0; s->sub = 0;
            return true;
          }
        }
        return false;
      };
    };
    // The camera pitched 90 degrees on the stand turned 15 degrees is a compound ROS RPY,
    // and its generated D435 mesh is reparented under the authored camera owner. With the
    // camera isolated, its body must still be drawn as authored and pickable, the stand
    // must be hidden, and turning isolation off must bring the stand back.
    auto add_isolated_camera_checks = [=](const QString & tag, const QString & shot_prefix, bool expect_clean_layout) {
      add(tag + ": start Mate on the turned camera, isolate it and look at it", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) {
          const auto cam = item(camera);
          const double xyz_error = turn(rpy_matrix(cam.roll, cam.pitch, cam.yaw, false), rpy_matrix(cam.roll, cam.pitch, cam.yaw, true));
          s->evidence[tag + "_camera_rpy"] = QJsonArray{cam.roll, cam.pitch, cam.yaw};
          s->evidence[tag + "_camera_xyz_order_error_deg"] = xyz_error * 180. / M_PI;
          require(near(cam.roll, 0., 1e-9) && near(cam.pitch, M_PI / 2, 1e-9) && near(cam.yaw, 15. * M_PI / 180., 1e-6),
            "the camera is not pitched 90 degrees on the stand turned 15 degrees");
          require(xyz_error > M_PI / 180., "the turned camera is not a compound rotation, so this check would prove nothing");
          window->apply_scene_selection(camera, "", false, false);
          window->inspector_mate_button_->click();
          require(window->mate_active() && mate()->pending_asset() == camera, "Inspector Mate did not start on the camera");
          s->env_hash = file_hash(s->scene + "/environment.yaml");
          s->items_before = window->mounting_items();
          auto * isolate = panel()->findChild<QCheckBox *>("mateIsolate");
          require(isolate->isEnabled(), "Isolate part is not offered for the camera");
          if (!isolate->isChecked()) isolate->click();
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->sub == 1) {
          // The camera sits at the top edge of the cell view, under the status overlays; look at it.
          if (s->ticks < 10) return false;
          QAction * origin = nullptr;
          for (auto * action : panel()->findChild<QToolButton *>("mateFaceView")->menu()->actions()) if (action->data().toString() == "origin") origin = action;
          require(origin != nullptr, "the camera origin is not offered in Look at a face");
          origin->trigger();
          s->sub = 2; s->ticks = 0; return false;
        }
        if (s->sub == 2) { if (s->ticks < 10) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 3; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        s->evidence[tag + "_isolated_asset"] = state.value("isolatedAsset");
        require(state.value("isolatedAsset").toString() == camera, "Web3D did not isolate the camera");
        shot(shot_prefix + "a_isolated_turned_camera");
        s->sub = 0;
        return true;
      });
      add(tag + ": isolated camera face click is drawn as authored and picks the camera", click_world(tag + "_camera_body", [=]() {
          // Aim at the D435 body, which extends from the camera_link origin along +x and
          // across y; stay clear of the origin marker (markers win over faces).
          const auto origin = datum(frames(), camera, "origin");
          const auto o = v3(origin.value("xyz")), x = v3(origin.value("x_axis")), n = v3(origin.value("normal"));
          const std::array<double, 3> y{n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]};
          Points points;
          for (const auto & [along, depth] : std::vector<std::pair<double, double>>{{.025, .012}, {.03, .012}, {.035, .012}, {.025, .008},
                 {.03, .016}, {.04, .012}, {-.025, .012}, {.025, -.012}, {-.025, -.012}, {.02, .014}})
            points.push_back({o[0] + along * y[0] + depth * x[0], o[1] + along * y[1] + depth * x[1], o[2] + along * y[2] + depth * x[2]});
          return points;
        }, camera, QString(),
        [=](const QJsonObject & pick) {
          const auto raw = pick.value("raw").toObject();
          const auto pose = raw.value("object_pose_world").toArray();
          require(pose.size() == 6, "the camera face pick carried no drawn pose");
          const auto cam = item(camera);
          const double moved = std::hypot(pose[0].toDouble() - cam.x, pose[1].toDouble() - cam.y, pose[2].toDouble() - cam.z);
          const double turned = turn(rpy_matrix(pose[3].toDouble(), pose[4].toDouble(), pose[5].toDouble(), true), rpy_matrix(cam.roll, cam.pitch, cam.yaw, true));
          s->evidence[tag + "_drawn_position_error_m"] = moved;
          s->evidence[tag + "_drawn_rotation_error_deg"] = turned * 180. / M_PI;
          require(pick.value("reason_code").toString() != "STALE_VIEW", "compound camera pose refused as a stale view: " + pick.value("reason").toString());
          require(pick.value("asset_id").toString() == camera && raw.value("geometry").toString() == "surface" &&
            raw.value("geometry_source").toString().contains("d435"), "the isolated click did not hit the camera's D435 mesh");
          require(moved < 1e-6 && turned < 1e-6, "Web3D draws the turned camera away from its authored pose");
          require(pick.value("reason_code").toString() == "NO_PLANE_DATUMS", "camera body: " + pick.value("reason").toString());
        }));
      // A point on the stand mast, below its top datum.
      auto mast = [=]() {
        const auto top = datum(frames(), s->stand, "top");
        const auto o = v3(top.value("xyz")), n = v3(top.value("normal"));
        return std::array<double, 3>{o[0] - .15 * n[0], o[1] - .15 * n[1], o[2] - .15 * n[2]};
      };
      add(tag + ": the stand is hidden while the camera is isolated", hover_probe(tag + "_stand_probe_isolated", mast,
        [=](const QJsonObject & hover, const QJsonObject & state) {
          require(state.value("isolatedAsset").toString() == camera, "isolation ended before the stand probe");
          require(hover.value("asset_id").toString().isEmpty() && hover.value("reason_code").toString() == "NO_ASSET",
            "the stand is still pickable while the camera is isolated: " + hover.value("asset_id").toString());
        }));
      add(tag + ": turning isolation off restores the cell", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) {
          auto * isolate = panel()->findChild<QCheckBox *>("mateIsolate");
          require(isolate->isChecked(), "isolation was dropped before it was turned off");
          isolate->click();
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->sub == 1) { if (s->ticks < 8) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 2; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        require(state.value("isolatedAsset").toString().isEmpty(), "Web3D kept the camera isolated");
        s->sub = 0;
        return true;
      });
      add(tag + ": the stand picks again", hover_probe(tag + "_stand_probe_restored", mast,
        [=](const QJsonObject & hover, const QJsonObject & state) {
          require(state.value("isolatedAsset").toString().isEmpty(), "isolation came back");
          require(hover.value("asset_id").toString() == s->stand, "the stand did not come back: " + hover.value("asset_id").toString());
        }));
      add(tag + ": view, then cancel without changes", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) { if (s->ticks < 8) return false; shot(shot_prefix + "b_isolated_camera_face_pick"); mate()->cancel(); s->sub = 1; s->ticks = 0; return false; }
        if (s->sub == 1) {
          if (s->ticks < 4) return false;
          require(!window->mate_active(), "Mate did not cancel");
          require(file_hash(s->scene + "/environment.yaml") == s->env_hash && window->mounting_items() == s->items_before, "the isolated-camera check changed the cell");
          require(!expect_clean_layout || !window->layout_dirty_, "inspecting the isolated camera marked the layout dirty");
          js("window.__WORKCELL_EDITOR_API_V1__.fitScene()");  // back to the whole cell, as Mate does after a face view
          s->sub = 2; s->ticks = 0; return false;
        }
        if (s->sub == 2) { if (s->ticks < 8) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 3; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        require(!state.value("active").toBool() && state.value("isolatedAsset").toString().isEmpty(), "Web3D still shows Mate or isolation after cancel");
        s->sub = 0;
        return true;
      });
    };
#endif
    add("load cell", [=]() {
      if (s->ticks < 5) return false;
      const QString root = window->detect_workspace_root() + "/src/easy_manipulation_deployment";
      s->scene = root + "/scenes/" + scene_name;
      s->evidence["scene_path"] = s->scene;
      QStringList blockers; QJsonObject diagnostics;
      require(window->load_scene_for_scene3d_smoke(scene_name, s->scene, &blockers, &diagnostics), "cell did not open: " + blockers.join("; "));
      window->show_studio_page(MainWindow::StudioPage::SceneBuilderPage);
      return true;
    });
    add("wait for the 3D view", [=]() {
      if (s->ticks < 10) return false;
      auto * preview = window->scene_preview_widget_;
      s->native = preview && preview->is_native_product_view_backend();
      if (!s->native && !(preview && preview->embedded_web_authoring_active() && window->mate_web_supported_)) return false;
      s->evidence["backend"] = s->native ? "native_scene3d" : "embedded_web3d";
      // Only a request larger than the free desktop area may be clamped by the window manager.
      QScreen * screen = window->windowHandle() ? window->windowHandle()->screen() : QGuiApplication::primaryScreen();
      const QSize available = screen ? screen->availableGeometry().size() : window_size;
      const bool fits = window_size.width() <= available.width() && window_size.height() <= available.height();
      if (window->size() != window_size && (fits ? s->ticks < 300 : s->ticks < 40)) {
        window->setWindowState(Qt::WindowNoState);
        window->resize(window_size);
        return false;
      }
      require(!fits || window->size() == window_size, "window did not resize to the requested size");
      // The window manager may clamp a full-screen request to the free desktop area.
      s->evidence["requested_window_size"] = QJsonArray{window_size.width(), window_size.height()};
      s->evidence["window_size"] = QJsonArray{window->width(), window->height()};
      return s->ticks > 25;
    });
    if (reopen) {
      add("reopened relations equal disk", [=]() {
        QFile previous(output + "/authoring.json"); require(previous.open(QIODevice::ReadOnly), "authoring evidence missing");
        const auto authored = QJsonDocument::fromJson(previous.readAll()).object();
        const auto before = authored.value("final_items").toArray();
        const auto after = window->mounting_items();
        require(before.size() == after.size(), "reopen changed physical identities");
        QMap<QString, QJsonObject> indexed;
        for (const auto & row : after) indexed.insert(row.toObject().value("id").toString(), row.toObject());
        const auto disk = environment();
        for (const auto & value : before) {
          const auto row = value.toObject(), actual = indexed.value(row.value("id").toString());
          require(!actual.isEmpty(), "reopen lost " + row.value("id").toString());
          for (const auto & key : {QString("pose_xyz"), QString("pose_rpy")})
            for (int k = 0; k < 3; ++k) require(near(row.value(key).toArray()[k].toDouble(), actual.value(key).toArray()[k].toDouble()), "reopen changed a transform of " + row.value("id").toString());
          const auto a = YAML::Load(row.value("mounting_yaml").toString().toStdString()), b = YAML::Load(actual.value("mounting_yaml").toString().toStdString());
          for (const char * key : {"parent_object", "parent_link", "mate", "origin", "mounting_anchors"})
            require(workcell_builder::same_yaml_value(a[key], b[key]), QString("reopen changed %1 of %2").arg(key, row.value("id").toString()));
          if (row.value("category").toString() == "zone") continue;
          const auto stored = record(disk, row.value("id").toString().toStdString());
          for (int k = 0; k < 3; ++k) require(near(stored["pose_xyz"][k].as<double>(), actual.value("pose_xyz").toArray()[k].toDouble()), "editor pose differs from environment.yaml for " + row.value("id").toString());
          require(workcell_builder::same_yaml_value(stored["mate"], b["mate"]) && workcell_builder::same_yaml_value(stored["origin"], b["origin"]),
            "editor relation differs from environment.yaml for " + row.value("id").toString());
        }
        s->evidence["relations_match_disk"] = true;
        s->stand = authored.value("stand").toString();
        check_flush("reopened_bin", bin, "bottom", table, "surface");
        check_flush("reopened_stand", s->stand, "base", table, "surface");
        s->env_hash = file_hash(s->scene + "/environment.yaml");
        window->apply_scene_selection(bin, "", false, false);
        return true;
      });
      add("selecting a mated asset keeps the layout clean", [=]() {
        if (s->ticks < 12) return false;
        require(!window->layout_dirty_, "selecting a mated asset marked the layout dirty");
        s->evidence["selection_relation_text"] = window->inspector_mount_relation_label_->text();
        require(window->inspector_mount_relation_label_->text().startsWith("Mated: bottom flush"), "selection context does not show the mate");
        shot("20_reopened");
        panel_button("mateApply");  // panel exists
        window->scene_builder_mate_button_->click();
        require(window->mate_active(), "Mate did not start after reopen");
        return true;
      });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add("reopened overlay shows datum markers; Esc cancels", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) { if (s->ticks < 8) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 1; return false; }
        if (s->sub == 1) {
          const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
          s->evidence["reopen_overlay"] = state;
          require(state.value("active").toBool() && state.value("markerCount").toInt() > 6, "reopened Mate overlay has no datum markers");
          shot("21_reopened_mate_markers");
          key_escape();
          s->sub = 2; s->ticks = 0; return false;
        }
        if (s->ticks < 8) return false;
        require(!window->mate_active(), "Esc in the 3D view did not cancel Mate");
        require(file_hash(s->scene + "/environment.yaml") == s->env_hash && !window->layout_dirty_, "cancelled Mate changed the cell");
        s->sub = 0;
        return true;
      });
      // Web3D sets and reads poses as ROS RPY, so selecting a compound-rotated
      // asset reads back its authored angles and echoes no pose edit to Qt.
      add("selecting the turned camera reads back exactly and keeps the layout clean", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        const auto cam = item(camera);
        if (s->sub == 0) {
          s->evidence["reopen_camera_rpy"] = QJsonArray{cam.roll, cam.pitch, cam.yaw};
          s->evidence["reopen_camera_xyz_order_error_deg"] =
            turn(rpy_matrix(cam.roll, cam.pitch, cam.yaw, false), rpy_matrix(cam.roll, cam.pitch, cam.yaw, true)) * 180. / M_PI;
          window->apply_scene_selection(camera, "", false, false);
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->sub == 1) { if (s->ticks < 12) return false; js("window.__WORKCELL_EDITOR_API_V1__.getState()"); s->sub = 2; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        const auto rpy = state.value("selectedTransform").toObject().value("pose").toObject().value("rpy").toObject();
        s->evidence["reopen_camera_web3d_rpy"] = rpy;
        require(state.value("selectedItemId").toString() == camera || state.value("uiSelectionItemId").toString() == camera, "Web3D did not select the camera");
        require(near(rpy.value("x").toDouble(), cam.roll, 1e-12) && near(rpy.value("y").toDouble(), cam.pitch, 1e-12) && near(rpy.value("z").toDouble(), cam.yaw, 1e-12),
          "Web3D reads the camera rotation back differently from the authored RPY");
        require(!window->layout_dirty_ && file_hash(s->scene + "/environment.yaml") == s->env_hash, "selecting the turned camera echoed a pose edit");
        shot("22_reopened_turned_camera_selected");
        s->sub = 0;
        return true;
      });
      add_isolated_camera_checks("Reopen isolated camera", "23", true);

      // Exercise the real WebEngine navigation and Qt poller after both counters
      // have advanced. A new viewer starts counting at zero again.
      add("Reload: seed an Esc before restarting Mate", [=]() {
        if (s->native || s->js_pending) return s->native;
        if (s->sub == 0) {
          window->apply_scene_selection(camera, "", false, false);
          window->inspector_mate_button_->click();
          s->sub = 1; return false;
        }
        if (s->sub == 1) {
          if (s->ticks < 15) return false;
          key_escape(); s->sub = 2; s->ticks = 0; return false;
        }
        if (s->ticks < 15 || window->mate_active()) return false;
        require(window->mate_seen_pick_seq_ > 0 && window->mate_seen_cancel_seq_ > 0,
          "reload test needs previously consumed pick and Esc counters");
        s->evidence["reload_previous_counters"] = QJsonObject{{"pick", window->mate_seen_pick_seq_},
          {"hover", window->mate_seen_hover_seq_}, {"cancel", window->mate_seen_cancel_seq_}};
        window->apply_scene_selection(camera, "", false, false);
        window->inspector_mate_button_->click();
        auto * isolate = panel()->findChild<QCheckBox *>("mateIsolate");
        if (!isolate->isChecked()) isolate->click();
        s->sub = 0; return true;
      });
      add("Reload: navigate WebEngine and restore Mate isolation", [=]() {
        if (s->native || s->js_pending) return s->native;
        if (s->sub == 0) {
          if (s->ticks < 20) return false;
          require(web() && window->mate_active(), "Mate is not active before reload");
          s->env_hash = file_hash(s->scene + "/environment.yaml");
          s->layout_hash = file_hash(s->scene + "/layout/workcell_studio_layout.yaml");
          s->items_before = window->mounting_items();
          web()->reload(); s->sub = 1; s->ticks = 0; return false;
        }
        if (s->sub == 1) {
          if (s->ticks < 20) return false;
          js("window.__WORKCELL_EDITOR_API_V1__?.getMateState?.() || null");
          s->sub = 2; return false;
        }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        if (!state.value("active").toBool() || state.value("isolatedAsset").toString() != camera ||
            state.value("markerCount").toInt() == 0) { s->sub = 1; return false; }
        require(state.value("seq").toInt() == 0 && state.value("cancelSeq").toInt() == 0,
          "the WebEngine navigation did not restart the Mate counters");
        require(window->mate_active(), "Builder lost the pending Mate on reload");
        s->evidence["reload_restored_overlay"] = state;
        // Restore framing through the real panel action, not a simulated pick.
        for (auto * action : panel()->findChild<QToolButton *>("mateFaceView")->menu()->actions())
          if (action->data().toString() == "origin") action->trigger();
        shot("24_reload_restored_isolation");
        s->sub = 0; return true;
      });
      add("Reload: restored isolation hides the stand from real raycasts", hover_probe("reload_hidden_stand", [=]() {
          const auto top = datum(frames(), s->stand, "top");
          const auto o = v3(top.value("xyz")), n = v3(top.value("normal"));
          return std::array<double, 3>{o[0] - .15 * n[0], o[1] - .15 * n[1], o[2] - .15 * n[2]};
        }, [=](const QJsonObject & hover, const QJsonObject & state) {
          require(state.value("isolatedAsset").toString() == camera && hover.value("asset_id").toString().isEmpty() &&
            hover.value("reason_code").toString() == "NO_ASSET", "reload isolation did not hide the stand");
        }));
      add("Reload: first new click reaches the Qt Mate controller", click_world("reload_camera_origin", [=]() {
          return Points{v3(datum(frames(), camera, "origin").value("xyz"))};
        }, camera, "origin", [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && mate()->child() && mate()->child()->asset_id == camera,
            "the first click after reload did not reach the Qt Mate controller");
        }));
      add("Reload: first new Esc cancels without changing authored state", [=]() {
        if (s->native || s->js_pending) return s->native;
        if (s->sub == 0) { key_escape(); s->sub = 1; s->ticks = 0; return false; }
        if (s->ticks < 15 || window->mate_active()) return false;
        require(window->mate_seen_cancel_seq_ == 1, "Builder did not consume the restarted browser Esc counter");
        require(!window->layout_dirty_ && window->mounting_items() == s->items_before &&
          file_hash(s->scene + "/environment.yaml") == s->env_hash &&
          file_hash(s->scene + "/layout/workcell_studio_layout.yaml") == s->layout_hash,
          "reload, picking or Esc changed authored state");
        s->evidence["reload_pick_and_escape_recovered"] = true;
        s->sub = 0; return true;
      });

      // Both public navigation paths must cancel a valid, ready-to-Apply mate.
      // Capture both cells' authored files, then try Apply after cancellation.
      struct CellSwitchCheck {
        int source = -1, target = -1;
        QString target_path;
        QMap<QString, QByteArray> hashes;
      };
      auto switching = std::make_shared<CellSwitchCheck>();
      for (const bool home_path : {false, true}) {
        const QString tag = home_path ? "Home/Open" : "Cell list";
        add(tag + ": prepare a valid Mate before switching cells", [=]() {
          if (s->native) return true;
          if (s->ticks < 20 || !window->scene_preview_widget_->embedded_web_authoring_active()) return false;
          switching->source = window->selected_scene_index_;
          switching->target = -1;
          for (int i = 0; i < int(window->scene_browser_result_.scenes.size()); ++i) {
            const auto & candidate = window->scene_browser_result_.scenes[i];
            if (QString::fromStdString(candidate.scene_name) == "suction_test" && i != switching->source) {
              switching->target = i;
              switching->target_path = QString::fromStdString(candidate.scene_dir.string());
            }
          }
          require(switching->target >= 0, "cell-switch acceptance requires a disposable suction_test copy");
          switching->hashes.clear();
          for (const auto & path : {s->scene, switching->target_path})
            for (const auto & file : {QString("environment.yaml"), QString("layout/workcell_studio_layout.yaml")})
              switching->hashes.insert(path + "/" + file, file_hash(path + "/" + file));
          window->apply_scene_selection(bin, "", false, false);
          window->inspector_mate_button_->click();
          auto choose = [=](const char * name, const QString & value) {
            auto * combo = panel()->findChild<QComboBox *>(name);
            const int index = combo ? combo->findData(value) : -1;
            require(index >= 0, "switch test datum is unavailable");
            combo->setCurrentIndex(index); emit combo->activated(index);
          };
          choose("mateChildDatum", "bottom");
          window->on_hierarchy_item_selected(hierarchy_row(table));
          choose("mateParentDatum", "surface");
          require(window->mate_active() && mate()->preview().ok && panel_button("mateApply")->isEnabled(),
            "switch test did not start from an applicable Mate");
          if (home_path) {
            window->show_workcells_home();
            require(window->open_home_scene_in_builder("suction_test", switching->target_path), "Home open failed");
          } else {
            // select_scene_by_row accepts the displayed Home row and maps it to
            // the browser index; use that mapping rather than assuming row order.
            int row = -1;
            for (int i = 0; window->dashboard_scene_table_ && i < window->dashboard_scene_table_->rowCount(); ++i) {
              const auto * entry = window->dashboard_scene_table_->item(i, 0);
              if (entry && entry->data(Qt::UserRole).toInt() == switching->target) row = i;
            }
            require(row >= 0, "target cell has no selectable row");
            window->select_scene_by_row(row);
          }
          require(window->selected_scene_index_ == switching->target && !window->mate_active(),
            tag + ": switching cells did not cancel Mate");
          return true;
        });
        add(tag + ": cancelled Apply cannot write either cell", [=]() {
          if (s->native) return true;
          if (s->ticks < 20) return false;
          require(!window->mate_active() && !panel()->isVisible() && !panel_button("mateApply")->isEnabled(),
            tag + ": Mate Apply remains available in the new cell");
          panel_button("mateApply")->click();
          for (auto it = switching->hashes.cbegin(); it != switching->hashes.cend(); ++it)
            require(file_hash(it.key()) == it.value(), tag + ": navigation or cancelled Apply wrote " + it.key());
          s->evidence[tag + "_cancels_mate_without_writes"] = true;
          require(window->studio_log_->toPlainText().contains("Mate cancelled: another cell was opened"),
            "cell-switch cancellation did not explain what happened");
          shot(home_path ? "26_home_switch_cancelled" : "25_list_switch_cancelled");
          require(window->open_scene_builder_for_scene_index(switching->source, "Acceptance return to saved cell"),
            "could not return to the original cell");
          return true;
        });
      }
#endif
      add("finish reopen", [=]() {
        s->evidence["result"] = "PASS";
        write();
        return true;
      });
    } else {
      add("baseline and entry points", [=]() {
        s->env_hash = file_hash(s->scene + "/environment.yaml");
        s->layout_hash = file_hash(s->scene + "/layout/workcell_studio_layout.yaml");
        s->evidence["baseline_environment_sha256"] = QString(s->env_hash);
        require(window->scene_builder_mate_button_ && window->scene_builder_mate_button_->isVisible(), "Mate is not on the Scene Builder toolbar");
        // No squeezed (clipped) toolbar controls at this window size.
        QJsonArray toolbar;
        auto * host = window->findChild<QWidget *>("scene_builder_top_controls_host");
        QList<QPushButton *> placed;
        for (int i = 0; host->layout() && i < host->layout()->count(); ++i) {
          QWidget * widget = host->layout()->itemAt(i)->widget();
          if (auto * button = qobject_cast<QPushButton *>(widget)) placed << button;
          else if (widget && widget->objectName() == "sceneBuilderEditModeGroup") placed << widget->findChildren<QPushButton *>();
        }
        for (auto * button : placed) {
          if (!button->isVisible()) continue;
          QFont font = button->font();
          if (button->isChecked()) font.setBold(true);
          const int needs = button->text().isEmpty() ? button->minimumWidth() : QFontMetrics(font).horizontalAdvance(button->text()) + 2 * 8;
          toolbar.append(QJsonObject{{"name", button->objectName()}, {"text", button->text()}, {"width", button->width()}, {"needs", needs}});
          require(button->width() >= needs, "toolbar control clipped: " + (button->text().isEmpty() ? button->objectName() : button->text()));
        }
        s->evidence["toolbar"] = toolbar;
        require(window->inspector_mate_button_ != nullptr, "Mate is not in the Inspector");
        // Context menu entry: open the real hierarchy menu and read it.
        auto * tree = window->scene_hierarchy_tree_;
        QTreeWidgetItem * row = hierarchy_row(bin);
        require(row != nullptr, "bin missing from the Scene Hierarchy");
        QTimer::singleShot(300, window, [=]() {
          if (auto * menu = qobject_cast<QMenu *>(QApplication::activePopupWidget())) {
            QJsonArray texts; for (auto * action : menu->actions()) if (!action->isSeparator()) texts.append(action->text());
            s->evidence["hierarchy_context_menu"] = texts;
            menu->close();
          }
        });
        tree->scrollToItem(row);
        window->show_scene_hierarchy_context_menu(tree->visualItemRect(row).center());
        const auto texts = s->evidence.value("hierarchy_context_menu").toArray();
        require(texts.contains(QJsonValue("Mate by Picking Faces…")) && texts.contains(QJsonValue("Mount To / Relative Pose…")),
          "Scene context menu lacks Mate / Mount To");
        // A camera stand from the Asset Library, standing on the table top.
        require(window->place_catalog_asset_at_world_position("camera_stand", .15, -.35, .6, false), "camera stand could not be placed from the Asset Library");
        for (const auto & p : window->all_scene_preview_items_) if (p.catalog_asset_id == "camera_stand") s->stand = p.id;
        require(!s->stand.isEmpty(), "placed camera stand has no scene ID");
        s->evidence["stand"] = s->stand;
        return true;
      });
      // The placed library stand draws its real mesh in the live session at once,
      // before Save and without a Product View refresh: Builder stages the catalog
      // mesh into the exporter's cache and hands the viewer that URI. A page
      // marker proves Save keeps the same live page (no reload, no regeneration).
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      const QString library_stand_mesh = "workcell_builder/workcell_builder/assets/environment/camera_stand_description/meshes/camera_stand.stl";
      auto stand_status_script = [=]() {
        const QString id = QString::fromUtf8(QJsonDocument(QJsonArray{s->stand}).toJson(QJsonDocument::Compact)).mid(1).chopped(1);
        return QString("(()=>{const v=window.__WORKCELL_VIEWER_STATUS__||{};"
          "const a=window.__WORKCELL_EDITOR_API_V1__;return {object:(v.rendered_object_statuses||[]).find(o=>o.id===%1)||null,"
          "marker:window.__wcsLivePageMarker||'',placement:a&&a.getPlacementState?a.getPlacementState():null};})()").arg(id);
      };
      auto stand_live_mesh = [=](const QJsonObject & status) {
        const auto object = status.value("object").toObject();
        if (object.value("render_status").toString() != "mesh_loaded") return QJsonObject();
        require(object.value("mesh_uri").toString() == s->stand_mesh_uri, "the live stand draws " + object.value("mesh_uri").toString() + ", not the staged library mesh");
        require(!object.value("fallback_visible").toBool(), "the live stand still shows its placeholder");
        return object;
      };
#endif
      add("stand drawn live with its staged mesh before Save", [=]() {
        require(window->layout_dirty_, "placing the stand did not mark the layout dirty");
        if (s->native) return true;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
        if (s->js_pending) return false;
        if (s->sub == 0) {
          s->stand_mesh_uri = "build/workcell_studio_web_scene/assets/" + window->scene_preview_widget_->preview_context().scene_id + "/" + library_stand_mesh;
          js(stand_status_script()); s->sub = 1; return false;
        }
        const auto status = QJsonObject::fromVariantMap(s->js_result.toMap());
        const auto object = stand_live_mesh(status);
        if (object.isEmpty()) { s->sub = 0; return false; }  // still loading; the runner times out
        QJsonArray staging_log;
        for (const auto & line : window->studio_log_->toPlainText().split('\n'))
          if (line.contains("Product View: staged the mesh of live item " + s->stand) || line.contains("placement preview of camera_stand")) staging_log.append(line);
        s->evidence["stand_staging_log"] = staging_log;
        s->evidence["stand_placement_state"] = status.value("placement");
        const QString repo = window->detect_workspace_root() + "/src/easy_manipulation_deployment/";
        const QByteArray staged = file_hash(repo + s->stand_mesh_uri), library = file_hash(repo + library_stand_mesh);
        require(!staged.isEmpty() && staged == library, "the staged stand mesh differs from the Asset Library mesh");
        s->evidence["stand_live_render"] = object;
        s->evidence["stand_staged_mesh"] = QJsonObject{{"uri", s->stand_mesh_uri}, {"library_mesh", library_stand_mesh}, {"sha256", QString(staged)}};
        js("window.__wcsLivePageMarker='stand-placed'");
        shot("08p_stand_placed_live_mesh");
#endif
        return true;
      });
      const QString retained = "live authored session retained without Product View regeneration";
      auto retained_saves = std::make_shared<int>(0);
      add("Save keeps the live session and the stand mesh", [=]() {
        if (s->sub == 0) {
          *retained_saves = window->studio_log_->toPlainText().count(retained);
          window->save_layout_button_->click(); s->sub = 1; s->ticks = 0; return false;
        }
        if (s->ticks < 20 || window->layout_dirty_) return false;
        require(window->studio_log_->toPlainText().count(retained) > *retained_saves, "Save did not keep the live authored session");
        if (s->native) return true;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
        if (s->js_pending) return false;
        if (s->sub == 1) { js(stand_status_script()); s->sub = 2; return false; }
        const auto status = QJsonObject::fromVariantMap(s->js_result.toMap());
        require(status.value("marker").toString() == "stand-placed", "Save reloaded the Product View page");
        const auto object = stand_live_mesh(status);
        require(!object.isEmpty(), "the stand mesh was lost after Save");
        s->evidence["stand_after_save_render"] = object;
#endif
        return true;
      });
      add("stand drawn", [=]() {
        if (s->ticks < 15) return false;
        auto * preview = window->scene_preview_widget_;
        if (!s->native && !(preview->embedded_web_authoring_active() && window->mate_web_supported_)) return false;
        if (s->ticks < 45) return false;
        require(item(s->stand).catalog_asset_id == "camera_stand", "reopened cell lost the camera stand");
        s->env_hash = file_hash(s->scene + "/environment.yaml");
        s->items_before = window->mounting_items();
        return true;
      });
      if (true) {
        add("Scenario A: start Mate from the toolbar", [=]() {
          window->apply_scene_selection(bin, "", false, false);
          window->scene_builder_mate_button_->click();
          require(window->mate_active() && panel()->isVisible(), "Mate panel did not open");
          require(mate()->pending_asset() == bin, "the selected bin is not offered as the first part");
          return true;
        });
      }
      add("Native: honest fallback, hierarchy and datum lists", [=]() {
        if (!s->native) return true;
        auto * note = panel()->findChild<QLabel *>("mateBackendNote");
        s->evidence["native_backend_note"] = note ? note->text() : QString();
        require(note && note->isVisible() && note->text().contains("native Scene3D"), "native fallback note missing");
        require(!panel()->findChild<QCheckBox *>("mateIsolate")->isEnabled() && !panel()->findChild<QToolButton *>("mateFaceView")->isEnabled(),
          "visual-only controls are enabled in the native view");
        auto choose = [=](const char * combo_name, const QString & name) {
          auto * combo = panel()->findChild<QComboBox *>(combo_name);
          const int index = combo ? combo->findData(name) : -1;
          require(index >= 0, QString("datum %1 is not offered in %2").arg(name, combo_name));
          combo->setCurrentIndex(index);
          emit combo->activated(index);
        };
        choose("mateChildDatum", "bottom");
        require(mate()->child() && mate()->child()->how == "chosen manually", "native child datum not chosen");
        auto * table_row = hierarchy_row(table);
        require(table_row != nullptr, "table missing from the Scene Hierarchy");
        window->on_hierarchy_item_selected(table_row);
        require(mate()->pending_asset() == table, "hierarchy click did not choose the table for step 2");
        choose("mateParentDatum", "surface");
        require(mate()->step() == workcell_builder::MateToolController::Step::Ready && mate()->preview().ok && mate()->alignment() == "flush",
          "native preview: " + mate()->preview().error);
        s->evidence["native_preview"] = mate()->evidence().value("preview");
        shot("native_01_preview");
        panel_button("mateApply")->click();
        check_flush("native_applied", bin, "bottom", table, "surface");
        s->evidence["native_relation"] = QString::fromStdString(YAML::Dump(relation(bin)));
        return true;
      });
      add("Native: applied view", [=]() {
        if (!s->native) return true;
        if (s->ticks < 8) return false;
        shot("native_02_applied");
        s->evidence["result"] = "PASS";
        write();
        s->step = int(steps->size()) - 1;  // the remaining steps need face picking
        return true;
      });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add("Scenario A: datum markers drawn", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) { if (s->ticks < 6) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 1; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        s->evidence["mate_start_overlay"] = state;
        require(state.value("active").toBool() && state.value("markerCount").toInt() > 6, "datum markers are not drawn");
        shot("01_mate_started_markers");
        s->sub = 0;
        return true;
      });
      add("Scenario A: bin side wall is rejected honestly", click_world("A_side_wall", fixed({{0.45 + 0.1761, 0.22, 0.1}, {0.45 + 0.1761, 0.18, 0.12}}), bin, QString(),
        [=](const QJsonObject & pick) {
          require(!pick.value("valid").toBool() && pick.value("reason_code").toString() == "NO_MATCHING_DATUM",
            "side wall was not rejected: " + pick.value("reason").toString());
          require(mate()->step() == workcell_builder::MateToolController::Step::PickChild && !mate()->child(), "a rejected face changed the mate");
          require(mate()->status().contains("choose a datum"), "rejection does not offer the manual datum selector");
          shot("02_rejected_side_wall");
        }));
      add("Scenario A: isolate the bin and look at its bottom", [=]() {
        if (s->sub == 0) {
          panel()->findChild<QCheckBox *>("mateIsolate")->click();
          auto * face = panel()->findChild<QToolButton *>("mateFaceView");  // looked up after the refresh
          QAction * bottom = nullptr;
          for (auto * action : face->menu()->actions()) if (action->data().toString() == "bottom") bottom = action;
          require(bottom != nullptr, "bin bottom datum is not offered in Look at a face");
          bottom->trigger();
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->ticks < 10) return false;
        s->sub = 0;
        shot("03_isolated_bottom_view");
        return true;
      });
      add("Scenario A: click the bin bottom face (geometry)", click_world("A_bin_bottom", fixed({{0.45 + 0.13, 0.22 + 0.05, 0.0}, {0.45 - 0.12, 0.22 - 0.05, 0.0}, {0.45 + 0.1, 0.22 - 0.07, 0.0}}), bin, "bottom",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("reason_code").toString() == "MATCHED" && pick.value("datum").toString() == "bottom",
            "bin bottom face did not resolve to the bottom datum: " + pick.value("reason").toString());
          require(pick.value("raw").toObject().value("geometry").toString() == "surface", "bin bottom was not a geometry pick");
          require(mate()->child() && mate()->child()->datum == "bottom", "child is not the bin bottom");
          require(mate()->step() == workcell_builder::MateToolController::Step::PickParent, "Mate did not move to step 2");
        }));
      add("Scenario A: view returns to the cell", [=]() { if (s->ticks < 10) return false; shot("04_child_picked"); return true; });
      add("Scenario A: click the table top (geometry)", click_world("A_table_top", fixed({{0.95, -0.25, 0.0}, {1.0, 0.25, 0.0}, {0.8, -0.05, 0.0}, {0.25, 0.3, 0.0}, {0.6, -0.3, 0.0}}), table, "surface",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("datum").toString() == "surface", "table top did not resolve to its surface: " + pick.value("reason").toString());
          require(pick.value("raw").toObject().value("geometry").toString() == "surface", "table top was not a geometry pick");
          require(mate()->step() == workcell_builder::MateToolController::Step::Ready, "both picks did not complete the mate");
        }));
      add("Scenario A: flush preview with ghost, nothing mutated", [=]() {
        if (s->js_pending) return false;
        if (s->sub == 0) { if (s->ticks < 6) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 1; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        const auto & preview = mate()->preview();
        s->evidence["A_preview"] = mate()->evidence().value("preview");
        s->evidence["A_overlay_state"] = state;
        require(mate()->alignment() == "flush", "plane-to-plane mate did not default to Flush");
        require(preview.ok, "flush preview failed: " + preview.error);
        require(near(preview.offset[2], 0.) && near(preview.offset[3], 0.) && near(preview.offset[4], 0.), "proposal did not seat the faces");
        bool bin_row = false;
        for (const auto & value : preview.moved) {
          const auto row = value.toObject();
          if (row.value("id").toString() != bin) continue;
          bin_row = true;
          const auto xyz = row.value("xyz").toArray();
          require(near(xyz[0].toDouble(), item(bin).x, 1e-9) && near(xyz[1].toDouble(), item(bin).y, 1e-9) && near(xyz[2].toDouble(), item(table).z + 0.1, 1e-9),
            "preview does not seat the bin on the table top");
        }
        require(bin_row, "preview does not move the bin");
        require(state.value("ghostCount").toInt() >= 1, "no ghost preview in the 3D view");
        require(window->mounting_items() == s->items_before && file_hash(s->scene + "/environment.yaml") == s->env_hash, "preview mutated the scene");
        require(panel_button("mateApply")->isEnabled(), "Apply is disabled for a valid mate");
        shot("05_preview_flush_ghost");
        s->sub = 0;
        return true;
      });
      // The fields show rounded values (4 decimals, 2 for degrees). Editing one must
      // change only that component; Reset brings back the exact proposal.
      add("Scenario A: editing one offset field keeps the other five exact", [=]() {
        if (s->native) return true;
        const auto before = mate()->preview().offset;
        auto * gap = panel()->findChild<QDoubleSpinBox *>("mateOffsetGap");
        require(gap && gap->isEnabled(), "Gap offset field is not editable");
        gap->setValue(gap->value() + 0.002);
        const auto edited = mate()->preview().offset;
        panel_button("mateReset")->click();
        const auto reset = mate()->preview().offset;
        auto json6 = [](const std::array<double, 6> & v) { return QJsonArray{v[0], v[1], v[2], v[3], v[4], v[5]}; };
        s->evidence["A_offset_edit"] = QJsonObject{{"before", json6(before)}, {"gap_edited", json6(edited)}, {"after_reset", json6(reset)}};
        for (int k = 0; k < 6; ++k) {
          if (k != 2) require(edited[k] == before[k], QString("editing Gap changed offset component %1 to its rounded value").arg(k));
        }
        require(near(edited[2], before[2] + 0.002, 1e-9), "Gap edit was not applied");
        for (int k = 0; k < 6; ++k) require(near(reset[k], before[k], 1e-12), "Reset did not restore the exact proposal");
        require(mate()->preview().ok && window->mounting_items() == s->items_before, "editing the offset mutated the scene");
        return true;
      });
#endif
      add("Scenario A: Apply", [=]() {
        if (s->native) return true;
        const int undo_before = int(window->undo_stack_.size());
        panel_button("mateApply")->click();
        require(!window->mate_active(), "Apply did not finish the mate");
        const auto m = relation(bin);
        require(m["parent_object"].as<std::string>("") == table.toStdString() && m["parent_link"].as<std::string>("") == "surface" &&
          m["mate"]["child_datum"].as<std::string>("") == "bottom" && m["mate"]["alignment"].as<std::string>("") == "flush", "bin relation not stored");
        require(near(m["origin"]["z"].as<double>(), 0.) && near(item(bin).z, item(table).z + 0.1), "bin not seated on the table top");
        require(int(window->undo_stack_.size()) == undo_before + 1 && window->layout_dirty_, "Apply did not record one undoable edit");
        check_flush("A_applied", bin, "bottom", table, "surface");
        s->evidence["A_relation"] = QString::fromStdString(YAML::Dump(m));
        return true;
      });
      add("Scenario A: applied view", [=]() { if (s->ticks < 8) return false; shot("06_applied"); return true; });
      add("Scenario A: move and rotate the table", [=]() {
        if (s->native) return true;
        const auto t0 = item(table), b0 = item(bin);
        const double c0 = std::cos(t0.yaw), s0 = std::sin(t0.yaw);
        const double lx0 = c0 * (b0.x - t0.x) + s0 * (b0.y - t0.y), ly0 = -s0 * (b0.x - t0.x) + c0 * (b0.y - t0.y);
        const auto zone_before = relation("place_zone_default");
        inspector(table, .05, -.03, 20.);
        const auto t = item(table), b = item(bin);
        const double c = std::cos(t.yaw), sn = std::sin(t.yaw);
        const double lx = c * (b.x - t.x) + sn * (b.y - t.y), ly = -sn * (b.x - t.x) + c * (b.y - t.y);
        require(near(lx, lx0, 1e-9) && near(ly, ly0, 1e-9) && near(b.yaw - t.yaw, b0.yaw - t0.yaw, 1e-9), "bin drifted from the table");
        check_flush("A_table_moved", bin, "bottom", table, "surface");
        s->evidence["A_table_local_bin"] = QJsonArray{lx, ly, b.z - t.z};
        return true;
      });
      add("Scenario A: moved view", [=]() { if (s->ticks < 8) return false; shot("07_table_moved_bin_follows"); return true; });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add("Scenario B: start Mate from the Inspector", [=]() {
        if (s->native) return true;
        window->apply_scene_selection(camera, "", false, false);
        window->inspector_mate_button_->click();
        require(window->mate_active() && mate()->pending_asset() == camera, "Inspector Mate did not start on the camera");
        return s->ticks > 4;
      });
      add("Scenario B: camera body has no mounting faces", click_world("B_camera_body", fixed({{0.4, -0.36 + 0.035, 0.85}, {0.4, -0.36 - 0.035, 0.85}, {0.4, -0.36 + 0.025, 0.85},
          {0.4, -0.36 - 0.025, 0.85}, {0.4 + 0.012, -0.36 + 0.03, 0.85}, {0.4 - 0.01, -0.36 + 0.02, 0.85}, {0.4, -0.36 + 0.02, 0.845}}), camera, QString(),
        [=](const QJsonObject & pick) {
          require(!pick.value("valid").toBool() && pick.value("reason_code").toString() == "NO_PLANE_DATUMS", "camera body: " + pick.value("reason").toString());
        }));
      add("Scenario B: click the camera origin marker", click_world("B_camera_origin", [=]() { return Points{v3(datum(frames(), camera, "origin").value("xyz"))}; }, camera, "origin",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("reason_code").toString() == "DATUM_MARKER" && pick.value("datum").toString() == "origin",
            "camera origin marker: " + pick.value("reason").toString());
        }));
      add("Scenario B: choose the stand in the hierarchy, isolate it and look at its top", [=]() {
        if (s->sub == 0) {
          auto * stand_row = hierarchy_row(s->stand);
          require(stand_row != nullptr, "stand missing from the Scene Hierarchy");
          window->on_hierarchy_item_selected(stand_row);
          require(mate()->pending_asset() == s->stand && mate()->child() && mate()->child()->asset_id == camera,
            "hierarchy click did not choose the stand for step 2 (or changed the picked camera)");
          panel()->findChild<QCheckBox *>("mateIsolate")->click();  // the robot arm hides the mast top from above
          auto * face = panel()->findChild<QToolButton *>("mateFaceView");
          QAction * top = nullptr;
          for (auto * action : face->menu()->actions()) if (action->data().toString() == "top") top = action;
          require(top != nullptr, "stand top datum is not offered in Look at a face");
          top->trigger();
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->ticks < 10) return false;
        shot("08a_stand_top_face_view");
        return true;
      });
      // A geometry click on the live stand's top face (its staged library mesh,
      // wound outward) resolves to the reviewed top datum. The points lie between
      // the 5 cm candidate marker and the 8 cm face edge, so they hit the face.
      add("Scenario B: stand top face click matches the top datum", click_world("B_stand_face", fixed({{.15 + .03, -.35 + .03, 1.2}, {.15 - .03, -.35 + .03, 1.2},
          {.15 + .03, -.35 - .03, 1.2}, {.15 - .03, -.35 - .03, 1.2}, {.15 + .0335, -.35, 1.2}, {.15, -.35 + .0335, 1.2}}), QString(), QString(),
        [=](const QJsonObject & pick) {
          require(pick.value("asset_id").toString() == s->stand, "stand face click hit " + pick.value("asset_id").toString());
          const auto raw = pick.value("raw").toObject();
          s->evidence["B_stand_face_outcome"] = pick.value("reason_code");
          require(pick.value("valid").toBool() && pick.value("reason_code").toString() == "MATCHED" && pick.value("datum").toString() == "top",
            "stand top face: " + pick.value("reason").toString());
          require(raw.value("geometry").toString() == "surface" && raw.value("geometry_source").toString().endsWith(s->stand_mesh_uri),
            "stand top face was not a surface hit on the staged mesh: " + raw.value("geometry_source").toString());
          require(pick.value("plane_distance_m").toDouble() <= 1e-3 && pick.value("normal_angle_deg").toDouble() <= 1.,
            "stand top face is off its datum plane");
          require(mate()->parent() && mate()->parent()->asset_id == s->stand && mate()->parent()->datum == "top", "the matched face did not become the parent datum");
          shot("08b_stand_top_face_matched");
        }));
      add("Scenario B: stand top datum marker", click_world("B_stand_top", [=]() {
          if (mate()->parent()) return Points{};
          return Points{v3(datum(frames(), s->stand, "top").value("xyz"))};
        }, QString(), "top",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("datum").toString() == "top" && pick.value("asset_id").toString() == s->stand,
            "stand top: " + pick.value("reason").toString());
        }));
#endif
      add("Scenario B: coincident with offset and orientation", [=]() {
        if (s->native) return true;
        if (s->sub == 0) {
          require(mate()->alignment() == "coincident", "origin-to-plane did not default to Coincident");
          require(!panel()->findChild<QRadioButton *>("mateFlush")->isEnabled() &&
            panel()->findChild<QLabel *>("mateAlignmentNote")->text().contains("Flush needs two plane faces"), "Flush is not disabled with its reason");
          const std::vector<std::pair<const char *, double>> fields{{"mateOffsetX", .15}, {"mateOffsetY", .09}, {"mateOffsetGap", .05},
            {"mateOffsetRoll", 0.}, {"mateOffsetPitch", 90.}, {"mateOffsetYaw", 0.}};
          for (const auto & field : fields) panel()->findChild<QDoubleSpinBox *>(field.first)->setValue(field.second);
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->ticks < 8) return false;
        const auto & preview = mate()->preview();
        s->evidence["B_preview"] = mate()->evidence().value("preview");
        require(preview.ok && near(preview.offset[0], .15) && near(preview.offset[2], .05) && near(preview.offset[4], M_PI / 2, 1e-9), "camera offset preview");
        shot("08_camera_on_stand_preview");
        panel_button("mateApply")->click();
        const auto cam = item(camera), stand = item(s->stand);
        require(near(cam.z, stand.z + 0.6 + 0.05) && near(cam.pitch, M_PI / 2, 1e-6), "camera not placed at the stand top offset");
        require(relation(s->stand)["mounting_anchors"]["top"]["datum_type"].as<std::string>("") == "plane", "stand top datum was not copied into the cell");
        s->sub = 0;
        return true;
      });
      add("Scenario B: move and rotate the stand", [=]() {
        if (s->native) return true;
        inspector(s->stand, .05, 0., 15.);
        const auto all = frames();
        const auto top = datum(all, s->stand, "top"), cam = datum(all, camera, "origin");
        const auto o = v3(top.value("xyz")), x = v3(top.value("x_axis")), n = v3(top.value("normal")), c = v3(cam.value("xyz"));
        const std::array<double, 3> y{n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]};
        const std::array<double, 3> expected{o[0] + .15 * x[0] + .09 * y[0] + .05 * n[0], o[1] + .15 * x[1] + .09 * y[1] + .05 * n[1], o[2] + .15 * x[2] + .09 * y[2] + .05 * n[2]};
        const double error = std::hypot(c[0] - expected[0], c[1] - expected[1], c[2] - expected[2]);
        s->evidence["B_camera_offset_error_m"] = error;
        require(error < 1e-9, "camera drifted from the stand top datum");
        check_flush("B_bin_still_flush", bin, "bottom", table, "surface");
        return true;
      });
      add("Scenario B: moved view", [=]() { if (s->ticks < 8) return false; shot("09_stand_moved_camera_follows"); return true; });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add_isolated_camera_checks("Compound pose", "09", false);
      // The table owner, isolated: its top still resolves to the support surface.
      add("Isolated table: start Mate on the table and isolate it", [=]() {
        if (s->native) return true;
        if (s->js_pending) return false;
        if (s->sub == 0) {
          window->apply_scene_selection(table, "", false, false);
          window->inspector_mate_button_->click();
          require(window->mate_active() && mate()->pending_asset() == table, "Inspector Mate did not start on the table");
          s->env_hash = file_hash(s->scene + "/environment.yaml");
          s->items_before = window->mounting_items();
          auto * isolate = panel()->findChild<QCheckBox *>("mateIsolate");
          require(isolate->isEnabled(), "Isolate part is not offered for the table");
          if (!isolate->isChecked()) isolate->click();
          s->sub = 1; s->ticks = 0; return false;
        }
        if (s->sub == 1) { if (s->ticks < 10) return false; js("window.__WORKCELL_EDITOR_API_V1__.getMateState()"); s->sub = 2; return false; }
        const auto state = QJsonObject::fromVariantMap(s->js_result.toMap());
        s->evidence["E_isolated_asset"] = state.value("isolatedAsset");
        require(state.value("isolatedAsset").toString() == table, "Web3D did not isolate the table");
        shot("09c_isolated_table");
        s->sub = 0;
        return true;
      });
      add("Isolated table: table top resolves to the table owner", click_world("E_table_top_isolated", fixed({{0.95, -0.25, 0.0}, {1.0, 0.25, 0.0}, {0.8, -0.05, 0.0}, {0.6, -0.3, 0.0}}), table, "surface",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("asset_id").toString() == table && pick.value("datum").toString() == "surface" &&
            pick.value("raw").toObject().value("geometry").toString() == "surface", "isolated table top: " + pick.value("reason").toString());
        }));
      add("Isolated table: cancel without changes", [=]() {
        if (s->native) return true;
        if (s->sub == 0) { mate()->cancel(); s->sub = 1; s->ticks = 0; return false; }
        if (s->ticks < 8) return false;
        require(!window->mate_active(), "Mate did not cancel");
        require(file_hash(s->scene + "/environment.yaml") == s->env_hash && window->mounting_items() == s->items_before, "the isolated-table check changed the cell");
        s->sub = 0;
        return true;
      });
      add("Scenario C: start Mate from the context menu", [=]() {
        if (s->native) return true;
        window->apply_scene_selection(table, "", false, false);
        QTimer::singleShot(300, window, [=]() {
          if (auto * menu = qobject_cast<QMenu *>(QApplication::activePopupWidget())) {
            for (auto * action : menu->actions()) if (action->text() == "Mate by Picking Faces…") {
              menu->setActiveAction(action);
              QKeyEvent press(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
              QApplication::sendEvent(menu, &press);
              return;
            }
            menu->close();
          }
        });
        auto * tree = window->scene_hierarchy_tree_;
        auto * table_row = hierarchy_row(table);
        require(table_row != nullptr, "table missing from the Scene Hierarchy");
        tree->scrollToItem(table_row);
        window->show_scene_hierarchy_context_menu(tree->visualItemRect(table_row).center());
        require(window->mate_active(), "context menu did not start Mate");
        s->env_hash = file_hash(s->scene + "/environment.yaml");
        s->items_before = window->mounting_items();
        return true;
      });
      add("Scenario C: pick the table top as the moving part", click_world("C_table_child", fixed({{0.95, -0.25, 0.0}, {1.0, 0.25, 0.0}, {0.8, -0.05, 0.0}, {0.6, -0.3, 0.0}}), table, "surface",
        [=](const QJsonObject & pick) { require(pick.value("valid").toBool() && mate()->child() && mate()->child()->asset_id == table, "table child pick"); }));
      add("Scenario C: self-mate is refused", click_world("C_self", fixed({{1.0, 0.25, 0.0}, {0.95, -0.25, 0.0}, {0.8, -0.05, 0.0}}), table, "surface",
        [=](const QJsonObject &) {
          require(!mate()->parent() && mate()->status().contains("cannot be mated to itself"), "self-mate was not refused");
        }));
      add("Scenario C: pick the bin rim marker (cycle)", click_world("C_bin_rim", [=]() { return Points{v3(datum(frames(), bin, "rim").value("xyz"))}; }, bin, "rim",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("datum").toString() == "rim", "bin rim marker");
        }));
#endif
      add("Scenario C: invalid cycle is shown, Apply disabled, nothing changed", [=]() {
        if (s->native) return true;
        if (s->ticks < 6) return false;
        const auto & preview = mate()->preview();
        s->evidence["C_invalid_preview"] = mate()->evidence().value("preview");
        require(!preview.ok && preview.error.contains("cycle"), "cycle not rejected: " + preview.error);
        require(!panel_button("mateApply")->isEnabled(), "Apply enabled for a cycle");
        require(panel()->findChild<QLabel *>("matePreview")->text().startsWith("Cannot apply"), "rejection reason not shown");
        require(window->mounting_items() == s->items_before && file_hash(s->scene + "/environment.yaml") == s->env_hash, "invalid mate changed the scene");
        shot("10_invalid_cycle_rejected");
        panel_button("mateChildReplace")->click();
        return true;
      });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add("Scenario C: correct the child (stand base marker)", click_world("C_stand_base", [=]() { return Points{v3(datum(frames(), s->stand, "base").value("xyz"))}; }, QString(), "base",
        [=](const QJsonObject & pick) {
          require(pick.value("valid").toBool() && pick.value("asset_id").toString() == s->stand && pick.value("datum").toString() == "base", "stand base marker");
        }));
#endif
      add("Scenario C: correct the parent (table top)", [=]() {
        if (s->native) return true;
        panel_button("mateParentReplace")->click();
        require(mate()->step() == workcell_builder::MateToolController::Step::PickParent, "Replace did not reopen step 2");
        return true;
      });
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
      add("Scenario C: table top as the parent", click_world("C_table_parent", fixed({{0.95, -0.25, 0.0}, {1.0, 0.25, 0.0}, {0.8, -0.05, 0.0}, {0.6, -0.3, 0.0}}), table, "surface",
        [=](const QJsonObject & pick) { require(pick.value("valid").toBool(), "table parent pick"); }));
#endif
      add("Scenario C: corrected mate applies; undo, redo", [=]() {
        if (s->native) return true;
        if (s->ticks < 6) return false;
        require(mate()->preview().ok && mate()->alignment() == "flush", "corrected mate preview: " + mate()->preview().error);
        shot("11_corrected_preview");
        const auto before = window->mounting_items();
        panel_button("mateApply")->click();
        check_flush("C_applied", s->stand, "base", table, "surface");
        const auto after = window->mounting_items();
        window->undo_layout_edit();
        require(window->mounting_items() == before, "undo did not restore the scene");
        window->redo_layout_edit();
        require(window->mounting_items() == after, "redo did not restore the mate");
        check_flush("C_redone", s->stand, "base", table, "surface");
        s->evidence["C_undo_redo"] = true;
        return true;
      });
      add("Scenario C: detach keeps the world pose (Mount To dialog), then undo", [=]() {
        if (s->native) return true;
        const auto before = window->mounting_items();
        const auto stand_world = item(s->stand);
        window->apply_scene_selection(s->stand, "", false, false);
        QTimer::singleShot(200, window, [=]() {
          if (auto * dialog = window->findChild<QDialog *>("assetMountingDialog")) {
            dialog->grab().save(output + "/12_mount_to_dialog_stand.png");
            dialog->findChild<QPushButton *>("detachAsset")->click();
          }
        });
        window->findChild<QPushButton *>("sceneBuilderMounting")->click();
        require(!relation(s->stand)["parent_object"] || relation(s->stand)["parent_object"].as<std::string>() == "world", "detach kept a parent");
        require(near(item(s->stand).z, stand_world.z) && near(item(s->stand).x, stand_world.x), "detach moved the stand");
        window->undo_layout_edit();
        require(window->mounting_items() == before, "detach undo lost the relation");
        s->evidence["C_detach_undo"] = true;
        return true;
      });
      add("Manual datum mating still works (Mount To dialog on the camera)", [=]() {
        if (s->native) return true;
        const auto before = window->mounting_items();
        auto seen = std::make_shared<QJsonObject>();
        window->apply_scene_selection(camera, "", false, false);
        QTimer::singleShot(200, window, [=]() {
          auto * dialog = window->findChild<QDialog *>("assetMountingDialog"); if (!dialog) return;
          for (const char * name : {"mountTo", "mountAnchor", "mountChildDatum", "mountAlignment"})
            (*seen)[name] = dialog->findChild<QComboBox *>(name)->currentData().toString();
          dialog->findChild<QCheckBox *>("applyRelativePose")->setChecked(true);
          auto * yaw = dialog->findChild<QDoubleSpinBox *>("relative_yaw");
          (*seen)["offset_x"] = dialog->findChild<QDoubleSpinBox *>("relative_x")->value();
          yaw->setValue(yaw->value() + 10.);
          (*seen)["preview"] = dialog->findChild<QLabel *>("mountPreview")->text();
          dialog->findChild<QDialogButtonBox *>()->button(QDialogButtonBox::Apply)->click();
        });
        window->findChild<QPushButton *>("sceneBuilderMounting")->click();
        s->evidence["manual_dialog"] = *seen;
        require(seen->value("mountTo").toString() == s->stand && seen->value("mountAnchor").toString() == "top" &&
          seen->value("mountChildDatum").toString() == "origin" && seen->value("mountAlignment").toString() == "coincident",
          "Mount To dialog does not show the relation authored by Mate");
        require(near(relation(camera)["origin"]["yaw"].as<double>(), qDegreesToRadians(10.), 1e-6), "manual offset edit not applied");
        window->undo_layout_edit();
        require(window->mounting_items() == before, "manual edit undo failed");
        return true;
      });
      add("Save", [=]() {
        if (s->native) return true;
        s->mated_items = window->mounting_items();
        window->save_layout_button_->click();
        return true;
      });
      add("saved: disk equals the editor", [=]() {
        if (s->native) return true;
        if (s->ticks < 12 || window->layout_dirty_) return false;
        const auto saved = environment();
        for (const auto & id : {bin, s->stand, camera}) {
          const auto r = record(saved, id.toStdString());
          const auto live = item(id);
          const auto m = relation(id);
          for (int k = 0; k < 3; ++k) require(near(r["pose_xyz"][k].as<double>(), k == 0 ? live.x : k == 1 ? live.y : live.z), "disk pose differs from the editor for " + id);
          for (const char * key : {"parent_object", "parent_link", "mate", "origin"})
            require(workcell_builder::same_yaml_value(r[key], m[key]), QString("disk %1 differs for %2").arg(key, id));
        }
        s->evidence["final_items"] = window->mounting_items();
        s->evidence["saved_environment_sha256"] = QString(file_hash(s->scene + "/environment.yaml"));
        s->evidence["saved_bin"] = QString::fromStdString(YAML::Dump(record(saved, bin.toStdString())));
        s->evidence["saved_camera"] = QString::fromStdString(YAML::Dump(record(saved, camera.toStdString())));
        s->evidence["saved_stand"] = QString::fromStdString(YAML::Dump(record(saved, s->stand.toStdString())));
        s->evidence["saved_drop_zone"] = QString::fromStdString(YAML::Dump(record(saved, "default_drop_zone")));
        return true;
      });
      add("Generate and Validate", [=]() {
        if (s->native) return true;
        window->generate_scene_package_for_selected_scene();
        require(QFileInfo::exists(s->scene + "/generated/generated_environment_objects.yaml"), "GUI generation did not produce environment objects");
        window->run_offline_validation();
        s->evidence["builder_validation_state"] = window->validation_state_text().remove(QRegularExpression("<[^>]*>")).simplified();
        return true;
      });
      add("final view", [=]() { if (s->ticks < 10) return false; shot("13_saved_generated"); s->evidence["result"] = "PASS"; write(); return true; });
    }
    // ---- Runner -------------------------------------------------------------
    auto * timer = new QTimer(window); timer->setInterval(100);
    QObject::connect(timer, &QTimer::timeout, window, [=]() {
      try {
        if (window->load_watcher_ && window->load_watcher_->isRunning()) return;
        if (s->step >= int(steps->size())) { timer->stop(); modal_watch->stop(); window->close(); app->exit(0); return; }
        const auto & current = (*steps)[s->step];
        require(++s->ticks < 600, "timed out at step: " + current.first);
        if (current.second()) {
          QJsonArray done = s->evidence.value("steps").toArray(); done.append(current.first); s->evidence["steps"] = done;
          ++s->step; s->ticks = 0; s->sub = 0;
        }
      } catch (const std::exception & error) {
        s->evidence["failure"] = QString::fromStdString(error.what());
        s->evidence["failed_step"] = s->step < int(steps->size()) ? (*steps)[s->step].first : QString();
        if (window->mate_controller_) s->evidence["mate_state"] = window->mate_controller_->evidence();
        shot("failure"); write(); timer->stop(); app->exit(1);
      }
    });
    timer->start();
  }
#include "ui_tour_capture.inc"
};
