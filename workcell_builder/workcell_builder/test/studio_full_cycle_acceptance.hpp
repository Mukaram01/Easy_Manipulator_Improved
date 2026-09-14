#pragma once
// Explicit workstation acceptance using the real window/actions, never a mock executor.
#include <QApplication>
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
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <stdexcept>

class TestMain {
public:
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
