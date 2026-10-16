#pragma once
// Explicit workstation acceptance using the real window/actions, never a mock executor.
#include <QApplication>
#include <QDialogButtonBox>
#include <QListWidget>
#include <QSettings>
#include "gui/new_cell_wizard.h"
#include "gui/task_intent_editor.h"
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QPushButton>
#include <QCheckBox>
#include <QTimer>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <stdexcept>

class TestMain {
public:
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
        dialog->findChild<QComboBox *>("mountAnchor")->setCurrentText(anchor);
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

  // Feature 4B: Task tab -> Save -> Generate -> "Check plans (no motion)"; --reopen proves persistence + recheck.
  static void start_multiobject_plan_acceptance(MainWindow * window, QApplication * app,
      const QString & output, const QString & recording, bool reopen) {
    struct Session { int phase=0, ticks=0; QString scene; QJsonObject evidence; };
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
                    std::abs(editor->findChild<QDoubleSpinBox *>("taskPlaceClearance")->value() - 0.005) < 1e-9 &&
                    std::abs(editor->findChild<QDoubleSpinBox *>("taskMaximumAge")->value() - 300.0) < 1e-9,
                    "saved task did not reopen identically");
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
          QFile report(evidence_dir + "/batch_plan_check.json");
          require(report.open(QIODevice::ReadOnly), "batch report missing");
          const auto batch = QJsonDocument::fromJson(report.readAll()).object();
          s->evidence["batch_report"] = evidence_dir + "/batch_plan_check.json";
          s->evidence["outcome_counts"] = batch.value("outcome_counts");
          require(batch.value("status").toString() == "CHECKED", "batch did not complete");
          require(!batch.value("execution_attempted").toBool() && !batch.value("motion_commanded").toBool(), "motion observed");
          for (int i = 0; i < window->scene_builder_inspector_tabs_->count(); ++i)
            if (window->scene_builder_inspector_tabs_->widget(i)->isAncestorOf(editor))
              window->scene_builder_inspector_tabs_->setCurrentIndex(i);
          s->phase = 6; s->ticks = 0; write(); break;
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
};
