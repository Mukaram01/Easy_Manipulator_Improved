// Linked to the actual colcon MainWindow objects by test_r11_canonical_readiness.py.
#include "gui/mainwindow.h"
#include <QApplication>
#include <QPushButton>
#include <QTabWidget>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <stdexcept>

void require(bool value, const char * message) { if (!value) throw std::runtime_error(message); }
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  MainWindow window(QString::fromLocal8Bit(argv[2]), "humble");
  workcell_builder::WorkcellStudioSceneInfo scene;
  scene.scene_name = "ur5_2f_test"; scene.scene_dir = argv[1];
  scene.has_package_xml = scene.has_launch_demo = scene.has_task_intent = true;
  scene.has_scene_urdf_xacro = true; scene.status = "READY";
  window.scene_browser_result_.scenes = {scene};
  window.selected_scene_index_ = 0;
  window.sync_selected_scene_state();
  window.layout_dirty_ = false;
  window.editable_layout_item_count_ = 1;
  const auto check_primary = [&]() {
    window.refresh_scene_workflow_rail();
    require(window.scene_workflow_recommendation_button_->isEnabled(), "primary disabled");
    require(window.resolve_recommended_workflow_action().enabled, "resolver promoted disabled action");
  };
  std::cerr << "Initial gate: " << window.selected_scene_readiness().blockers.join(" | ").toStdString() << "\n";
  require(window.selected_scene_preview_ready(), "valid scene without ignored merge cache blocked");
  const auto layout = std::filesystem::path(argv[1]) / "layout/workcell_studio_layout.yaml";
  const auto old_time = std::filesystem::last_write_time(layout);
  std::filesystem::last_write_time(layout, old_time + std::chrono::hours(24));
  require(window.selected_scene_preview_ready(), "touch made current scene stale");
  check_primary();
  require(window.resolve_recommended_workflow_action().token == "plan_simulate", "current scene does not recommend Plan");
  window.scene_workflow_recommendation_button_->click();
  require(window.studio_pages_->currentIndex() == static_cast<int>(MainWindow::StudioPage::PlanSimulatePage), "Run Next did not open Plan / Simulate");
  require(window.run_build_button_->isEnabled(), "current scene build/run is disabled");
  std::ifstream input(layout); const std::string original((std::istreambuf_iterator<char>(input)), {}); input.close();
  { auto edited = original; const auto offset = edited.find("Industrial Workbench");
    require(offset != std::string::npos, "canonical authored item absent");
    edited.replace(offset, std::string("Industrial Workbench").size(), "Edited Workbench");
    std::ofstream output(layout); output << edited; }
  require(!window.selected_scene_preview_ready(), "authored edit was not detected");
  require(!window.selected_scene_readiness().generation_current, "generation stayed current");
  check_primary();
  require(window.resolve_recommended_workflow_action().token == "generate_scene_package", "edit must promote Generate");
  for (const auto & step : window.scene_workflow_steps()) {
    if (step.label == "Generate Scene Package" || step.label == "Validate" || step.label == "RViz/MoveIt Fake-Hardware Launch")
      require(step.status != MainWindow::SceneWorkflowStepStatus::Done, "blocked prerequisite shows Done");
  }
  { std::ofstream output(layout); output << original; }
  const auto acceptance = std::filesystem::path(argv[1]) / "acceptance/generated_scene_acceptance.json";
  const auto receipt = acceptance.parent_path() / "generation_fingerprint.json";
  std::filesystem::copy_file(acceptance, receipt, std::filesystem::copy_options::overwrite_existing);
  std::filesystem::rename(acceptance, acceptance.string() + ".saved");
  check_primary();
  require(window.resolve_recommended_workflow_action().token == "validate_scene", "generated content must promote Validate");
  std::filesystem::rename(acceptance.string() + ".saved", acceptance);
  window.scene_browser_result_.scenes[0].launch_metadata_present = true;
  window.scene_browser_result_.scenes[0].launch_metadata_warning = "Required launch metadata is invalid";
  require(!window.selected_scene_preview_ready(), "unsafe launch gate weakened");
  check_primary();
  window.refresh_preview_launch_ui();
  require(!window.run_build_button_->isEnabled(), "blocked preview button remains enabled");
  require(window.scene_workflow_recommendation_label_->text().contains("Required launch metadata"), "blocker only in tooltip");
  auto * tabs = window.scene_builder_inspector_tabs_;
  require(tabs->widget(2)->isAncestorOf(window.scene_workflow_rail_label_), "Checks does not own workflow");
  require(tabs->widget(1)->isAncestorOf(window.task_intent_details_label_), "Task does not own intent");
  require(!tabs->widget(1)->isAncestorOf(window.scene_workflow_recommendation_button_), "Task owns Run Next");
  require(window.scene_builder_activity_summary("Product View HTTP optional browser resource: path='/favicon.ico' status=404", workcell_builder::StudioLogSeverity::Info).isEmpty(), "optional 404 replaced activity");
  require(window.scene_builder_activity_summary("Product View HTTP ERROR required resource: path='/scene.json' status=404", workcell_builder::StudioLogSeverity::Info).contains("Error"), "required 404 hidden");
  std::cout << "R1.1 Qt readiness, actions, ownership, HTTP: PASS\n";
}
