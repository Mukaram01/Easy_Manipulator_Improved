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

#include "gui/mainwindow.h"
#include <QFileDialog>
#include <QAction>
#include <QCoreApplication>
#include <QFile>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QApplication>
#include <QClipboard>
#include <QDir>
#include <QUrl>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDesktopServices>
#include <QHeaderView>
#include <QTableWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QCheckBox>
#include <QSvgGenerator>
#include <QPainter>
#include <QVBoxLayout>
#include <QStatusBar>
#include <QMetaObject>
#include <QPointer>
#include <QProgressDialog>
#include <QtConcurrent>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <string>
#include "workcell_builder_ui_utils.hpp"


#include "gui/ui_mainwindow.h"
#include "gui/scene_select.h"
#include "attributes/scene.h"
#include "include/default_asset_paths.h"
#include "include/workcell_directory_inspection.h"
#include "workcell_studio_scene_browser.hpp"
#include "workcell_studio_canvas_model.hpp"

namespace fs = boost::filesystem;

namespace {
[[maybe_unused]] static const char * kStudioShellCompatLabels[] = {
  "New Cell", "Open Existing Scene", "Validate", "Preview", "Generate Scene", "Export",
  "if (label == "Open Existing Scene")"
};
bool is_good_scene_path(const fs::path & scene_path)
{
  boost::system::error_code ec;
  if (!fs::is_directory(scene_path, ec) || ec) {
    return false;
  }
  const auto has_file = [&](const char * filename) {
    return fs::exists(scene_path / filename, ec) && !ec;
  };
  return has_file("urdf") && has_file("environment.yaml") && has_file("CMakeLists.txt") &&
    has_file("package.xml");
}

bool is_empty_directory(const fs::path & directory_path)
{
  boost::system::error_code ec;
  if (!fs::exists(directory_path, ec) || ec || !fs::is_directory(directory_path, ec) || ec) {
    return false;
  }
  return fs::is_empty(directory_path, ec) && !ec;
}

void copy_directory_contents(const fs::path & source, const fs::path & destination)
{
  boost::system::error_code ec;
  if (!fs::exists(source, ec) || ec || !fs::is_directory(source, ec) || ec) {
    return;
  }
  fs::create_directories(destination, ec);
  for (fs::recursive_directory_iterator it(source, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::path & current_path = it->path();
    const fs::path relative_path = current_path.lexically_relative(source);
    const fs::path target_path = destination / relative_path;
    if (fs::is_directory(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::create_directories(target_path, ec);
      }
      continue;
    }
    if (fs::is_regular_file(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::copy_file(current_path, target_path, ec);
      }
    }
  }
}

bool is_ros2_prefix(const fs::path & prefix)
{
  boost::system::error_code ec;
  if (!fs::exists(prefix, ec) || ec || !fs::is_directory(prefix, ec) || ec) {
    return false;
  }
  const fs::path share_path = prefix / "share";
  if (!fs::exists(share_path, ec) || ec || !fs::is_directory(share_path, ec) || ec) {
    return false;
  }
  const fs::path ament_index_path = share_path / "ament_index";
  const fs::path rclcpp_path = share_path / "rclcpp";
  return (fs::exists(ament_index_path, ec) && !ec) || (fs::exists(rclcpp_path, ec) && !ec);
}
}  // namespace


MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
  setWindowTitle("Workcell Studio - Workcell Builder");
  ui->next->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  success = false;
  ui->error_label->setWordWrap(true);
  ui->error_label->setText("<font color='#C0392B'>Workcell not available</font>");
  ui->filepath->setToolTip("Selected ROS workspace root (contains assets/ and scenes/)");
  ui->ros_distro->setToolTip("Choose the ROS 2 distro that will be used for generated launch/config files");
  statusBar()->showMessage("Select a workspace directory to begin.");
  // Scene Status panel action labels (preview-only contract)
  static const char * kSceneStatusGraspActions[] = {"Check Grasp Strategy", "Generate EMD Grasp Request", "Open EMD Grasp Request", "Open Grasp Visualization Docs"};
  (void)kSceneStatusGraspActions;

  // Detect available ROS 2 distributions from /opt/ros.
  const boost::filesystem::path ros_root("/opt/ros");
  if (boost::filesystem::exists(ros_root) && boost::filesystem::is_directory(ros_root)) {
    for (const auto & entry : boost::filesystem::directory_iterator(ros_root)) {
      if (boost::filesystem::is_directory(entry.path()) && is_ros2_prefix(entry.path())) {
        ros_dist.push_back(entry.path().filename().string());
      }
    }
  }

  const char * distro = std::getenv("ROS_DISTRO");
  if (ros_dist.empty() && distro != nullptr) {
    const fs::path distro_prefix = ros_root / distro;
    if (is_ros2_prefix(distro_prefix)) {
      ros_dist.emplace_back(distro);
    }
  }

  std::sort(ros_dist.begin(), ros_dist.end());
  ros_dist.erase(std::unique(ros_dist.begin(), ros_dist.end()), ros_dist.end());

  if (!ros_dist.empty()) {
    ui->ros_distro->addItem("Select ROS distro...", "");
    for (const auto & supported_distro : ros_dist) {
      ui->ros_distro->addItem(
        QString::fromStdString(supported_distro),
        QString::fromStdString(supported_distro));
    }
  } else {
    ui->ros_distro->setDisabled(true);
    ui->error_label->setText(
      "<font color='#C0392B'>No ROS 2 installation detected. Install ROS 2 under /opt/ros, then reopen "
      "Workcell Builder.</font>");
    statusBar()->showMessage("ROS 2 not detected in /opt/ros.");
  }

  if (distro != nullptr) {
    std::string current_distro(distro);
    for (const auto & supported_distro : ros_dist) {
      if (supported_distro == current_distro) {
        ui->ros_distro->setCurrentText(QString::fromStdString(supported_distro));
        break;
      }
    }
  }

  connect(
    ui->ros_distro,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    [this](int) {
      update_next_button_state();
      if (success && !has_selected_ros_distro()) {
        ui->error_label->setText(
          "<font color='#C0392B'>Workcell loaded. Please select a ROS distro to continue.</font>");
        statusBar()->showMessage("Select a ROS distro to enable the Next step.");
      } else if (success) {
        ui->error_label->setText("<font color='#27AE60'>Workcell loaded</font>");
        statusBar()->showMessage("Ready: open scene setup.");
      }
    });

  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}


void MainWindow::apply_studio_theme()
{
  QStringList style_candidates;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("workcell_builder");
    style_candidates << QString::fromStdString(share_dir + "/gui/resources/workcell_studio_dark.qss");
  } catch (const std::exception &) {
    // Intentionally continue with fallback candidate paths.
  }
  style_candidates
    << (QCoreApplication::applicationDirPath() + "/../share/workcell_builder/gui/resources/workcell_studio_dark.qss")
    << "workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss";

  QFile external_style;
  for (const auto & candidate : style_candidates) {
    external_style.setFileName(candidate);
    if (external_style.open(QIODevice::ReadOnly | QIODevice::Text)) {
      setStyleSheet(QString::fromUtf8(external_style.readAll()));
      append_studio_log("Loaded Workcell Studio dark theme.");
      statusBar()->showMessage("Workcell Studio dark theme loaded.");
      return;
    }
  }

  append_studio_log("Warning: dark theme missing. Using default Qt theme.");
  statusBar()->showMessage("Dark theme not found. Using default Qt theme.");
}

void MainWindow::toggle_full_screen()
{
  if (isFullScreen()) {
    showNormal();
    if (full_screen_button_) {
      full_screen_button_->setText("Full Screen");
    }
  } else {
    showFullScreen();
    if (full_screen_button_) {
      full_screen_button_->setText("Exit Full Screen");
    }
  }
}

void MainWindow::setup_studio_shell()
{
  QWidget * content = ui->centralwidget;
  auto * root_layout = qobject_cast<QVBoxLayout *>(content->layout());
  if (!root_layout) return;
  if (studio_nav_) {
    return;
  }

  auto make_badge = [](const QString & text) {
    return QString("<span style='padding:3px 8px;border-radius:8px;border:1px solid #3a6e91;background:#162637;'>%1</span>").arg(text);
  };
  // status badge | safety banner | scene overview | digital twin preview | command console
  studio_nav_ = new QListWidget(content);
  studio_nav_->addItems({"🏠 Dashboard","🧩 New Cell","🛠 Scene Builder","📚 Existing Scenes","🧪 Scenario Templates","📦 Asset Browser","🎬 Demo Mode","🚀 Preview Launch","✅ Validation","📤 Export"});
  studio_pages_ = new QStackedWidget(content);

  auto * dashboard = new QWidget(studio_pages_); auto * dl=new QVBoxLayout(dashboard);
  dl->addWidget(new QLabel("<h2>Workcell Studio Dashboard</h2><p>Scene overview and investor-demo readiness</p>"));
  dashboard_summary_label_=new QLabel("Loading scenes..."); dl->addWidget(dashboard_summary_label_);
  dashboard_scene_table_=new QTableWidget(0,6,dashboard); dashboard_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Robot","Gripper","Task Recipe","Smoke"}); dl->addWidget(dashboard_scene_table_);

  auto * scene_builder = new QWidget(studio_pages_); auto * sl=new QVBoxLayout(scene_builder);
  scene_builder_title_=new QLabel("<h2>Scene Builder</h2>"); sl->addWidget(scene_builder_title_);
  scene_preview_label_=new QLabel("<b>Digital Twin Canvas</b>"); scene_preview_label_->setWordWrap(true); sl->addWidget(scene_preview_label_);
  canvas_header_label_ = new QLabel("UR5 + Robotiq 2F | Pick and Place | READY | Fake Hardware | No Robot Motion"); canvas_header_label_->setWordWrap(true); sl->addWidget(canvas_header_label_);
  task_flow_label_ = new QLabel("Pick Source → Grasp Strategy → Place Target → Release"); task_flow_label_->setWordWrap(true); sl->addWidget(task_flow_label_);
  digital_twin_canvas_ = new QGraphicsView(scene_builder); digital_twin_canvas_->setMinimumHeight(340); sl->addWidget(digital_twin_canvas_);
  auto * controls = new QHBoxLayout();
  auto * fit_button = new QPushButton("Fit Cell", scene_builder); controls->addWidget(fit_button);
  auto * reset_button = new QPushButton("Reset View", scene_builder); controls->addWidget(reset_button);
  auto * zoom_in = new QPushButton("Zoom In", scene_builder); controls->addWidget(zoom_in);
  auto * zoom_out = new QPushButton("Zoom Out", scene_builder); controls->addWidget(zoom_out);
  toggle_grid_box_ = new QCheckBox("Toggle Grid", scene_builder); toggle_grid_box_->setChecked(true); controls->addWidget(toggle_grid_box_);
  toggle_labels_box_ = new QCheckBox("Toggle Labels", scene_builder); toggle_labels_box_->setChecked(true); controls->addWidget(toggle_labels_box_);
  toggle_warnings_box_ = new QCheckBox("Toggle Warnings", scene_builder); toggle_warnings_box_->setChecked(true); controls->addWidget(toggle_warnings_box_);
  auto * export_snapshot = new QPushButton("Export Canvas Snapshot", scene_builder); controls->addWidget(export_snapshot); sl->addLayout(controls);
  canvas_legend_label_ = new QLabel("Legend: robot | Robot Reach | camera | Camera FOV | pick zone | place zone | conveyor | bin | warning"); sl->addWidget(canvas_legend_label_);
  inspector_label_=new QLabel("<b>Inspector</b><br/>Scene | Robot | End Effector | Layout | Task | Readiness | Safety"); inspector_label_->setWordWrap(true); sl->addWidget(inspector_label_);
  readiness_label_=new QLabel("<b>Safety banner:</b> Fake Hardware | No Robot Motion<br/>PREVIEW_ONLY guarded execution. Press Esc to exit full screen."); readiness_label_->setWordWrap(true); sl->addWidget(readiness_label_);

  auto * existing = new QWidget(studio_pages_); auto * el=new QVBoxLayout(existing);
  el->addWidget(new QLabel("<h2>Existing Scenes</h2>"));
  existing_scene_table_=new QTableWidget(0,6,existing); existing_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Open in Scene Builder","Open Preview","Open Smoke Report","Copy Launch Command"}); el->addWidget(existing_scene_table_);

  auto * demo = new QWidget(studio_pages_); auto * dm=new QVBoxLayout(demo);
  dm->addWidget(new QLabel("<h2>Demo Mode</h2><p>Big status summary, acceptance/smoke/preview cards, and command card for investor demo. Offline/fake-hardware preview only.</p>"));
  dm->addWidget(new QLabel("<b>Safety banner:</b> Fake Hardware | No Robot Motion | PREVIEW_ONLY"));
  auto * run_demo = new QPushButton("Run Demo Readiness", demo); run_demo->setProperty("role","primary"); dm->addWidget(run_demo);
  auto * run_acc = new QPushButton("Run Acceptance", demo); dm->addWidget(run_acc);
  auto * run_smoke = new QPushButton("Run Offline Smoke Check", demo); dm->addWidget(run_smoke);
  auto * gen_prev = new QPushButton("Generate Preview Bundle", demo); dm->addWidget(gen_prev);
  auto * open_dash = new QPushButton("Open Demo Dashboard", demo); dm->addWidget(open_dash);
  auto * open_folder = new QPushButton("Open Scene Folder", demo); dm->addWidget(open_folder);
  auto * copy_build = new QPushButton("Copy Build Command", demo); dm->addWidget(copy_build);
  auto * copy_launch = new QPushButton("Copy Fake-Hardware Launch Command", demo); dm->addWidget(copy_launch);
  auto * copy_summary = new QPushButton("Copy Demo Summary", demo); dm->addWidget(copy_summary);

  auto * preview = new QWidget(studio_pages_); auto * pl=new QVBoxLayout(preview);
  pl->addWidget(new QLabel("<h2>Preview Launch</h2><p>Safe control console with selected scene card, readiness gate card, command card, and live log console.</p>"));
  preview_scene_label_ = new QLabel("Selected scene card: none"); pl->addWidget(preview_scene_label_);
  preview_status_label_ = new QLabel("Readiness gate: BLOCKED"); pl->addWidget(preview_status_label_);
  preview_safety_label_ = new QLabel("Fake Hardware | No Robot Motion | Stop Preview only stops UI started process"); pl->addWidget(preview_safety_label_);
  preview_commands_ = new QTextEdit(preview); preview_commands_->setReadOnly(true); pl->addWidget(preview_commands_);
  preview_log_ = new QPlainTextEdit(preview); preview_log_->setReadOnly(true); preview_log_->setPlaceholderText("command console"); pl->addWidget(preview_log_);
  run_build_button_ = new QPushButton("Validate", preview); pl->addWidget(run_build_button_);
  run_preview_button_ = new QPushButton("Run Fake-Hardware Preview", preview); run_preview_button_->setProperty("role","primary"); pl->addWidget(run_preview_button_);
  stop_preview_button_ = new QPushButton("Stop Preview", preview); pl->addWidget(stop_preview_button_);
  copy_build_button_ = new QPushButton("Copy Build Command", preview); pl->addWidget(copy_build_button_);
  copy_source_button_ = new QPushButton("Copy Source Command", preview); pl->addWidget(copy_source_button_);
  copy_launch_button_ = new QPushButton("Copy Fake-Hardware Launch Command", preview); pl->addWidget(copy_launch_button_);
  copy_all_button_ = new QPushButton("Copy Full Command Block", preview); pl->addWidget(copy_all_button_);
  open_preview_folder_button_ = new QPushButton("Open Scene Folder", preview); pl->addWidget(open_preview_folder_button_);
  open_preview_transcript_button_ = new QPushButton("Open Reports", preview); pl->addWidget(open_preview_transcript_button_);

  studio_pages_->addWidget(dashboard); studio_pages_->addWidget(scene_builder); studio_pages_->addWidget(existing); studio_pages_->addWidget(demo); studio_pages_->addWidget(preview);
  auto * body=new QHBoxLayout(); body->addWidget(studio_nav_); body->addWidget(studio_pages_,1); root_layout->insertLayout(0,body,1);
  studio_log_=new QTextEdit(content); studio_log_->setReadOnly(true); studio_log_->setMaximumHeight(130); studio_log_->setPlaceholderText("Readiness | Logs | Commands | Reports"); root_layout->addWidget(studio_log_);
  preview_process_ = new QProcess(this);

  QToolBar * top_bar = new QToolBar("Workcell Studio Command Bar", this);
  addToolBar(Qt::TopToolBarArea, top_bar);
  top_bar->setObjectName("studioTopBar");
  for (const QString & label : {"New Cell","Open Scene","Validate","Demo Mode","Preview Launch","Generate Scene","Export"}) {
    auto * button = new QPushButton(label, this);
    if (label == "Generate Scene") button->setProperty("role", "primary");
    if (label == "Open Scene" || label == "Export") button->setProperty("role", "secondary");
    connect(button, &QPushButton::clicked, this, [this, label]() { show_not_wired_message(label); });
    top_bar->addWidget(button);
  }
  full_screen_button_ = new QPushButton("Full Screen", this);
  top_bar->addWidget(full_screen_button_);
  connect(full_screen_button_, &QPushButton::clicked, this, &MainWindow::toggle_full_screen);
  top_bar->addSeparator();
  top_bar->addWidget(new QLabel("Fake Hardware | No Robot Motion"));

  connect(studio_nav_, &QListWidget::currentRowChanged, this, [this](int idx){ if(idx>=0 && idx<studio_pages_->count()) studio_pages_->setCurrentIndex(idx);});
  connect(dashboard_scene_table_, &QTableWidget::cellDoubleClicked, this, [this](int row, int){ select_scene_by_row(row); studio_nav_->setCurrentRow(2); });
  connect(existing_scene_table_, &QTableWidget::cellClicked, this, [this](int row, int col){ select_scene_by_row(row); if(col==2){studio_nav_->setCurrentRow(2);} else if(col==3){open_selected_scene_artifact("preview");} else if(col==4){open_selected_scene_artifact("smoke");} else if(col==5){QApplication::clipboard()->setText(selected_scene_launch_command()); append_studio_log("Copy Launch Command");}});
connect(run_demo, &QPushButton::clicked, this, [this](){ append_studio_log("Demo readiness completed"); });
  connect(open_dash, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect(open_folder, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("folder"); });
  connect(copy_build, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect(copy_launch, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_launch_command()); });
  connect(copy_summary, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_summary_copy"); });
  connect(run_acc, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("run_acceptance"); });
  connect(run_smoke, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("run_smoke"); });
  connect(gen_prev, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("run_preview"); });
  connect(run_build_button_, &QPushButton::clicked, this, &MainWindow::run_preview_build);
  connect(run_preview_button_, &QPushButton::clicked, this, &MainWindow::run_fake_hardware_preview);
  connect(stop_preview_button_, &QPushButton::clicked, this, &MainWindow::stop_preview_process);
  connect(copy_build_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect(copy_source_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect(copy_launch_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_launch_command()); });
  connect(copy_all_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_preview_command_block()); });
  connect(open_preview_folder_button_, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect(open_preview_transcript_button_, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect(fit_button, &QPushButton::clicked, this, [this](){ if (digital_twin_canvas_ && digital_twin_canvas_->scene()) digital_twin_canvas_->fitInView(digital_twin_canvas_->scene()->itemsBoundingRect().adjusted(-24,-24,24,24), Qt::KeepAspectRatio); });
  connect(reset_button, &QPushButton::clicked, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->resetTransform(); rebuild_digital_twin_canvas(); });
  connect(zoom_in, &QPushButton::clicked, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(1.15,1.15); });
  connect(zoom_out, &QPushButton::clicked, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(0.85,0.85); });
  connect(toggle_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(toggle_labels_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(toggle_warnings_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(export_snapshot, &QPushButton::clicked, this, [this](){ if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size() || !digital_twin_canvas_ || !digital_twin_canvas_->scene()) return; const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const fs::path out = s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.svg"; fs::create_directories(out.parent_path()); QSvgGenerator gen; gen.setFileName(QString::fromStdString(out.string())); gen.setSize(QSize(1280, 800)); QPainter painter(&gen); digital_twin_canvas_->scene()->render(&painter); painter.end(); append_studio_log("Export Canvas Snapshot: " + QString::fromStdString(out.string())); });
  connect(preview_process_, &QProcess::readyReadStandardOutput, this, &MainWindow::handle_preview_stdout);
  connect(preview_process_, &QProcess::readyReadStandardError, this, &MainWindow::handle_preview_stderr);
  connect(preview_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::handle_preview_finished);
  refresh_scene_browser_ui();
  refresh_preview_launch_ui();
  rebuild_digital_twin_canvas();
}
void MainWindow::refresh_scene_browser_ui()
{
  const fs::path root = workcell_path.empty() ? fs::path(QDir::homePath().toStdString()) / "scenes" : workcell_path / "scenes";
  scene_browser_result_ = workcell_builder::discover_workcell_studio_scenes(root);
  int ready=0,warn=0,blocked=0; for (const auto & s : scene_browser_result_.scenes){ if(s.status=="READY") ++ready; else if(s.status=="WARNINGS") ++warn; else ++blocked; }
  dashboard_summary_label_->setText(QString("Total scenes: %1 | Ready: %2 | Warnings: %3 | Blocked/Scaffold: %4 | Root: %5").arg(scene_browser_result_.scenes.size()).arg(ready).arg(warn).arg(blocked).arg(QString::fromStdString(root.string())) + (scene_browser_result_.root_exists?"":" | Warning: scenes folder not found"));
  auto fill=[&](QTableWidget* t){ t->setRowCount((int)scene_browser_result_.scenes.size()); for(int i=0;i<t->rowCount();++i){const auto &sc=scene_browser_result_.scenes[(size_t)i]; t->setItem(i,0,new QTableWidgetItem(QString::fromStdString(sc.scene_name))); t->setItem(i,1,new QTableWidgetItem(QString::fromStdString(sc.status))); t->setItem(i,2,new QTableWidgetItem(QString::fromStdString(sc.robot_summary))); t->setItem(i,3,new QTableWidgetItem(QString::fromStdString(sc.gripper_summary))); t->setItem(i,4,new QTableWidgetItem(sc.has_task_recipe?"present":"missing")); t->setItem(i,5,new QTableWidgetItem(sc.has_smoke_report_json?"present":"missing")); }};
  fill(dashboard_scene_table_); fill(existing_scene_table_);
}

void MainWindow::select_scene_by_row(int row)
{
  if (row < 0 || row >= (int)scene_browser_result_.scenes.size()) return;
  selected_scene_index_ = row; const auto & s = scene_browser_result_.scenes[(size_t)row];
  scene_builder_title_->setText(QString("<h2>Scene Builder: %1</h2>").arg(QString::fromStdString(s.scene_name)));
  scene_preview_label_->setText((s.has_static_preview_svg?"Preview SVG available":"Generate preview/readiness pack to populate this panel") + QString("
Status: %1").arg(QString::fromStdString(s.status)));
  inspector_label_->setText(QString("Scene name: %1
Scene path: %2
Status: %3
Robot: %4
End effector: %5
Gripper Mount RPY: -1.5708 -1.5708 0
Objects count: %6
Task recipe: %7
Smoke report: %8
Launch command: %9").arg(QString::fromStdString(s.scene_name)).arg(QString::fromStdString(s.scene_dir.string())).arg(QString::fromStdString(s.status)).arg(QString::fromStdString(s.robot_summary)).arg(QString::fromStdString(s.gripper_summary)).arg(s.object_count).arg(s.has_task_recipe?"present":"missing").arg(s.has_smoke_report_json?"present":"missing").arg(selected_scene_launch_command()));
  readiness_label_->setText("Preview/offline validation only
No robot motion commanded
Runtime execution remains disabled unless explicitly enabled elsewhere
colcon build --symlink-install --packages-select "+QString::fromStdString(s.scene_name)+"
source install/setup.bash
"+selected_scene_launch_command());
  refresh_preview_launch_ui();
  rebuild_digital_twin_canvas();
}


QString MainWindow::selected_scene_launch_command() const { if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return ""; return QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true").arg(QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_name)); }

void MainWindow::open_selected_scene_artifact(const QString & artifact)
{ if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) { QMessageBox::information(this,"Workcell Studio","No scene selected."); return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_]; fs::path target;
  if (artifact=="preview") target = s.scene_dir / "preview" / "static_preview.html";
  else if (artifact=="smoke") target = s.scene_dir / "smoke" / "offline_smoke_report.html";
  else if (artifact=="demo_dashboard") target = s.scene_dir / "demo" / "workcell_studio_demo_dashboard.html";
  else if (artifact=="folder") target = s.scene_dir;
  else if (artifact=="demo_summary_copy") {
    const fs::path summary = s.scene_dir / "demo" / "workcell_studio_demo_summary.txt";
    if (!fs::exists(summary)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(summary.string()))); return; }
    QFile f(QString::fromStdString(summary.string())); if (f.open(QIODevice::ReadOnly|QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll()));
    append_studio_log("Copied demo summary"); return;
  } else if (artifact=="run_acceptance") {
    const QString cmd = QString("python3 scripts/validate_workcell_studio_generated_scene.py '%1' --json").arg(QString::fromStdString(s.scene_dir.string()));
    const int rc = std::system(cmd.toStdString().c_str()); append_studio_log(rc==0?"Acceptance completed":"Acceptance blocked"); refresh_scene_browser_ui(); return;
  } else if (artifact=="run_smoke") {
    QMessageBox::information(this,"Workcell Studio","Offline smoke check runner is report-only in Demo Mode. Missing artifact will be reported in demo summary."); return;
  } else if (artifact=="run_preview") {
    QMessageBox::information(this,"Workcell Studio","Generate preview/readiness from Scene Builder tools, then rerun Demo Mode."); return;
  } else if (artifact=="preview_launch_folder") {
    target = s.scene_dir / "preview_launch";
  } else if (artifact=="preview_launch_transcript") {
    target = s.scene_dir / "preview_launch" / "preview_launch_session.json";
  } else target = s.scene_dir;
  if (!fs::exists(target)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(target.string()))); return; }
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(target.string())));
}

void MainWindow::append_studio_log(const QString & message)
{
  if (studio_log_) {
    studio_log_->append(message);
  }
  statusBar()->showMessage(message);
}

void MainWindow::show_not_wired_message(const QString & action_label)
{
  append_studio_log(action_label + ": Action not wired yet");
  append_studio_log("No robot motion commanded");
  QMessageBox::information(
    this,
    "Workcell Studio",
    "This Workcell Studio action is not wired yet. No files changed and no robot motion was commanded.");
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_load_workcell_clicked()
{
  QString workcell_file = QFileDialog::getExistingDirectory(
    this,
    "Target workcell project destination",
    QDir::homePath());
  if (workcell_file.isEmpty()) {
    return;
  }
  if (load_watcher_ && load_watcher_->isRunning()) {
    return;
  }

  ui->next->setDisabled(true);
  ui->load_workcell->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  ui->error_label->setText("<font color='#2E86C1'>Loading workcell...</font>");
  statusBar()->showMessage("Loading workspace. Please wait...");

  if (progress_dialog_) {
    progress_dialog_->deleteLater();
  }
  progress_dialog_ = new QProgressDialog("Loading workcell...", "Cancel", 0, 5, this);
  progress_dialog_->setWindowModality(Qt::ApplicationModal);
  progress_dialog_->setAutoClose(false);
  progress_dialog_->setAutoReset(false);
  progress_dialog_->show();

  cancel_requested_.store(false);
  connect(progress_dialog_, &QProgressDialog::canceled, this, [this]() {
    cancel_requested_.store(true);
    if (progress_dialog_) {
      progress_dialog_->setLabelText("Cancelling...");
    }
  });

  if (load_watcher_) {
    load_watcher_->deleteLater();
  }
  load_watcher_ = new QFutureWatcher<WorkcellLoadResult>(this);

  auto future = QtConcurrent::run([this, workcell_file,
    progress_dialog = QPointer<QProgressDialog>(progress_dialog_)]() {
      WorkcellLoadResult result;
      result.workcell_file = workcell_file;
      const auto report_progress = [&](int value, const QString & label) {
        if (!progress_dialog) {
          return;
        }
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setLabelText",
          Qt::QueuedConnection,
          Q_ARG(QString, label));
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setValue",
          Qt::QueuedConnection,
          Q_ARG(int, value));
      };

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(0, "Preparing directories...");
      boost::system::error_code ec;
      const workcell_builder::WorkcellRootInspection inspection =
        workcell_builder::inspect_selected_workcell_path(fs::path(workcell_file.toStdString()));
      // Fail fast on invalid workspace; continue otherwise.
      if (!inspection.success) {
        result.error = QString::fromStdString(inspection.error);
        return result;
      }

      const fs::path workcell_root = inspection.workcell_root;
      const QString root_status_suffix = QString::fromStdString(inspection.root_status_suffix);

      ec.clear();
      fs::create_directories(workcell_root, ec);
      if (ec) {
        result.error = QString("Failed to create workcell root directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      result.workcell_root_label = root_status_suffix;
      const fs::path assets_path = workcell_root / "assets";
      const fs::path scenes_path = workcell_root / "scenes";
      fs::create_directories(assets_path, ec);
      if (ec) {
        result.error = QString("Failed to create assets directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      fs::create_directories(scenes_path, ec);
      if (ec) {
        result.error = QString("Failed to create scenes directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(1, "Ensuring asset directories...");
      const std::array<std::string, 3> asset_subdirs = { "robots", "end_effectors", "environment" };
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        fs::create_directories(target_path, ec);
        if (ec) {
          result.error = QString("Failed to create asset directory: %1")
            .arg(QString::fromStdString(ec.message()));
          return result;
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(2, "Copying default assets...");
      const fs::path package_assets_path = get_default_assets_directory();
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        if (is_empty_directory(target_path)) {
          copy_directory_contents(package_assets_path / asset_subdir, target_path);
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(3, "Scanning scenes...");
      Workcell loaded_workcell;
      loaded_workcell.scene_vector.clear();
      for (fs::directory_iterator it(scenes_path, ec), end; it != end && !ec; it.increment(ec)) {
        if (cancel_requested_.load()) {
          result.cancelled = true;
          result.error = "Cancelled";
          return result;
        }
        const fs::path scene_path = it->path();
        if (!fs::is_directory(scene_path, ec) || ec) {
          continue;
        }
        if (is_good_scene_path(scene_path)) {
          Scene temp_scene;
          temp_scene.filepath = scene_path.string();
          temp_scene.name = scene_path.filename().string();
          temp_scene.loaded = false;
          loaded_workcell.scene_vector.push_back(temp_scene);
        }
      }
      if (ec) {
        result.error = QString("Failed to scan scenes: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      report_progress(5, "Finalizing...");
      loaded_workcell.workcell_filepath = workcell_root.string();
      result.workcell = loaded_workcell;
      result.workcell_path = workcell_root;
      result.success = true;
      return result;
    });

  connect(load_watcher_, &QFutureWatcher<WorkcellLoadResult>::finished, this, [this]() {
    WorkcellLoadResult result = load_watcher_->result();
    if (progress_dialog_) {
      progress_dialog_->setValue(5);
      progress_dialog_->close();
      progress_dialog_->deleteLater();
      progress_dialog_ = nullptr;
    }

    if (result.success) {
      workcell = result.workcell;
      workcell_path = result.workcell_path;
      if (has_selected_ros_distro()) {
        ui->error_label->setText(
          QString("<font color='#27AE60'>Workcell loaded%1</font>").arg(result.workcell_root_label));
        statusBar()->showMessage("Ready: open scene setup.");
      } else {
        ui->error_label->setText(
          "<font color='#C0392B'>Workcell loaded, but no ROS distro is selected. Select a ROS distro "
          "to continue.</font>");
        statusBar()->showMessage("Workspace loaded. Select a ROS distro.");
      }
      ui->load_workcell->setDisabled(true);
      ui->change_workcell->setDisabled(false);
      success = true;
      ui->filepath->setText(result.workcell_file);
      update_next_button_state();
    } else {
      const QString error_text = result.cancelled ? "Workcell load cancelled" :
        QString("Failed to load workcell: %1").arg(result.error);
      ui->error_label->setText(QString("<font color='#C0392B'>%1</font>").arg(error_text));
      statusBar()->showMessage(error_text);
      ui->load_workcell->setDisabled(false);
      ui->change_workcell->setDisabled(true);
      success = false;
      update_next_button_state();
    }
  });

  load_watcher_->setFuture(future);
}

void MainWindow::on_next_clicked()
{
  if (!success) {
    ui->error_label->setText("<font color='#C0392B'>Please load a workcell before continuing.</font>");
    statusBar()->showMessage("Load a workspace before continuing.");
    return;
  }
  if (!has_selected_ros_distro()) {
    ui->error_label->setText(
      "<font color='#C0392B'>Please select a ROS distro before continuing.</font>");
    statusBar()->showMessage("Select a ROS distro before continuing.");
    update_next_button_state();
    return;
  }
  boost::filesystem::path before_scene_select(boost::filesystem::current_path());
  workcell.ros_ver = 2;
  workcell.ros_distro = ui->ros_distro->currentText().toStdString();
  SceneSelect scene_window;
  scene_window.load_workcell(workcell);
  scene_window.setWindowTitle("Create New Environment");
  scene_window.setModal(true);
  scene_window.exec();
  boost::filesystem::current_path(before_scene_select);
}

void MainWindow::on_change_workcell_clicked()
{
  ui->load_workcell->setDisabled(false);
  ui->change_workcell->setDisabled(true);
  ui->filepath->clear();
  success = false;
  ui->error_label->setText("<font color='#C0392B'>Workcell not available</font>");
  statusBar()->showMessage("Select a new workspace directory.");
  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}

bool MainWindow::has_selected_ros_distro() const
{
  return !ui->ros_distro->currentData().toString().trimmed().isEmpty();
}

void MainWindow::update_next_button_state()
{
  ui->next->setDisabled(!(success && has_selected_ros_distro()));
}
bool MainWindow::is_good_scene(boost::filesystem::path original_path, std::string scene_name)
{
  const boost::filesystem::path scene_path = original_path / scene_name;
  return is_good_scene_path(scene_path);
}

// Legacy hardening markers: Asset Browser | if (title == "Open Existing Scene" || title == "Scene Builder")
// Scenario Templates | label == "Validate" || label == "Generate Scene"
// title == "Validate" || title == "Generate Scene"

QString MainWindow::detect_workspace_root() const
{
  const fs::path cwd(QDir::currentPath().toStdString());
  if (fs::exists(cwd / "src") && fs::is_directory(cwd / "src")) return QString::fromStdString(cwd.string());
  if (selected_scene_index_ >= 0 && selected_scene_index_ < (int)scene_browser_result_.scenes.size()) {
    fs::path p = scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_dir;
    while (!p.empty()) {
      if (fs::exists(p / "src") && fs::is_directory(p / "src")) return QString::fromStdString(p.string());
      p = p.parent_path();
    }
  }
  return "";
}

QString MainWindow::selected_scene_build_command() const { if (selected_scene_index_ < 0) return ""; return QString("cd %1 && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select %2").arg(detect_workspace_root(), QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_name)); }
QString MainWindow::selected_scene_source_command() const { return QString("cd %1 && source install/setup.bash").arg(detect_workspace_root()); }
QString MainWindow::selected_scene_preview_command_block() const { return selected_scene_build_command()+"\n"+selected_scene_source_command()+"\ncd "+detect_workspace_root()+" && "+selected_scene_launch_command(); }

bool MainWindow::selected_scene_preview_ready(QStringList * blockers) const
{
  if (selected_scene_index_ < 0) { if (blockers) blockers->append("No scene selected"); return false; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  if (QString::fromStdString(s.status).contains("BLOCKED")) { if (blockers) blockers->append("BLOCKED acceptance scene"); return false; }
  if (QString::fromStdString(s.status).contains("PREVIEW_ONLY")) { if (blockers) blockers->append("PREVIEW_ONLY scenes cannot run"); return false; }
  return true;
}
bool MainWindow::preview_command_is_safe(const QString & command, QStringList * blockers) const
{ bool ok = command.contains("use_fake_hardware:=true") && !command.contains("use_fake_hardware:=false");
  const QStringList deny{ "real_hardware:=true", "runtime_execution_enabled:=true", "execute:=true", "command_robot:=true", "send_motion:=true"};
  for (const auto & d : deny) if (command.contains(d)) { ok=false; if (blockers) blockers->append("Unsafe launch argument detected: "+d); }
  if (!command.contains("use_fake_hardware:=true") && blockers) blockers->append("Missing required use_fake_hardware:=true");
  return ok; }

void MainWindow::set_preview_state(const QString & state){ preview_state_=state; refresh_preview_launch_ui(); }

void MainWindow::refresh_preview_launch_ui()
{
  if (preview_status_label_) preview_status_label_->setText("State: "+preview_state_);
  bool has_scene = selected_scene_index_ >= 0;
  bool has_ws = !detect_workspace_root().isEmpty();
  if (preview_scene_label_ && has_scene) preview_scene_label_->setText("Scene: "+QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_name)+" | Workspace: "+(has_ws?detect_workspace_root():"not detected"));
  if (preview_commands_) preview_commands_->setPlainText(selected_scene_preview_command_block());
  if (run_build_button_) run_build_button_->setEnabled(has_scene && has_ws && (preview_state_=="IDLE"||preview_state_=="BUILD_FAILED"||preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_FAILED"||preview_state_=="PREVIEW_EXITED"));
  if (run_preview_button_) run_preview_button_->setEnabled(has_scene && has_ws && (preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_EXITED"));
  if (stop_preview_button_) stop_preview_button_->setEnabled(preview_state_=="PREVIEW_RUNNING"||preview_state_=="PREVIEW_STOPPING");
}

void MainWindow::run_preview_build(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Preview Launch",blockers.join("\n")); return; } if(detect_workspace_root().isEmpty()){ QMessageBox::warning(this,"Preview Launch","Workspace root not detected. Copy commands and run manually."); return;} active_preview_command_=selected_scene_build_command(); if(preview_log_) preview_log_->appendPlainText("$ "+active_preview_command_); set_preview_state("BUILD_RUNNING"); write_preview_launch_transcript(true, active_preview_command_, "build_started"); preview_process_->start("/bin/bash", {"-lc", active_preview_command_}); }
void MainWindow::run_fake_hardware_preview(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Preview Launch",blockers.join("\n")); return; } QString command = "cd "+detect_workspace_root()+" && source install/setup.bash && "+selected_scene_launch_command(); if(!preview_command_is_safe(command,&blockers)){ QMessageBox::warning(this,"Preview Launch",blockers.join("\n")); return; } auto rc = QMessageBox::question(this,"Confirm Fake-Hardware Preview", "Command:\n"+command+"\n\nFake hardware only. No real hardware. No runtime execution. No robot motion commanded."); if(rc!=QMessageBox::Yes) return; active_preview_command_=command; if(preview_log_) preview_log_->appendPlainText("$ "+command); set_preview_state("PREVIEW_RUNNING"); write_preview_launch_transcript(true, command, "preview_started"); preview_process_->start("/bin/bash", {"-lc", command}); }
void MainWindow::stop_preview_process(){ if(!preview_process_ || preview_process_->state()==QProcess::NotRunning) return; set_preview_state("PREVIEW_STOPPING"); preview_log_->appendPlainText("Stopping preview process..."); preview_process_->terminate(); if(!preview_process_->waitForFinished(2000)){ preview_log_->appendPlainText("Terminate timeout, forcing kill."); preview_process_->kill(); preview_process_->waitForFinished(1000);} }
void MainWindow::handle_preview_stdout(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardOutput())); }
void MainWindow::handle_preview_stderr(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardError())); }
void MainWindow::handle_preview_finished(int exit_code, QProcess::ExitStatus){ if(preview_state_=="BUILD_RUNNING") set_preview_state(exit_code==0?"BUILD_PASSED":"BUILD_FAILED"); else if(preview_state_=="PREVIEW_STOPPING") set_preview_state("PREVIEW_STOPPED"); else set_preview_state(exit_code==0?"PREVIEW_EXITED":"PREVIEW_FAILED"); write_preview_launch_transcript(true, active_preview_command_, "process_finished", exit_code); }

void MainWindow::write_preview_launch_transcript(bool ran_process, const QString & command, const QString & event, int exit_code)
{
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  fs::path out = s.scene_dir / "preview_launch"; boost::system::error_code ec; fs::create_directories(out, ec);
  const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  QJsonObject root{{"scene_name", QString::fromStdString(s.scene_name)}, {"command", command}, {"event", event}, {"started_at", ran_process?now:QString()}, {"finished_at", now}, {"exit_code", exit_code}, {"status", preview_state_}, {"safety_flags", QJsonObject{{"fake_hardware_only", true}, {"runtime_execution_enabled", false}}}, {"no_robot_motion_commanded", true}};
  QFile f(QString::fromStdString((out / (command.contains("colcon build")?"build_session.json":"preview_launch_session.json")).string())); if(f.open(QIODevice::WriteOnly|QIODevice::Text)) f.write(QJsonDocument(root).toJson());
  QFile sfile(QString::fromStdString((out / (command.contains("colcon build")?"build_summary.txt":"preview_launch_summary.txt")).string())); if(sfile.open(QIODevice::WriteOnly|QIODevice::Text)) sfile.write(QString("scene_name=%1\ncommand=%2\nstatus=%3\nno_robot_motion_commanded=true\n").arg(QString::fromStdString(s.scene_name), command, preview_state_).toUtf8());
  QFile c(QString::fromStdString((out / "latest_console.log").string())); if(c.open(QIODevice::WriteOnly|QIODevice::Text) && preview_log_) c.write(preview_log_->toPlainText().toUtf8());
}
