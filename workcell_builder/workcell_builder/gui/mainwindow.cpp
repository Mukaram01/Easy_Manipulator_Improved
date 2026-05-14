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
#include <QVBoxLayout>
#include <QStatusBar>
#include <QMetaObject>
#include <QPointer>
#include <QProgressDialog>
#include <QtConcurrent>
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

namespace fs = boost::filesystem;

namespace {
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
  QString style_path = QCoreApplication::applicationDirPath() + "/../share/workcell_builder/gui/resources/workcell_studio_dark.qss";
  QFile external_style(style_path);
  if (!external_style.open(QIODevice::ReadOnly | QIODevice::Text)) {
    external_style.setFileName("workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss");
    if (!external_style.open(QIODevice::ReadOnly | QIODevice::Text)) {
      return;
    }
  }
  setStyleSheet(QString::fromUtf8(external_style.readAll()));
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
  if (!root_layout) {
    return;
  }

  auto * top_bar = new QFrame(content);
  top_bar->setObjectName("studioTopBar");
  auto * top_layout = new QHBoxLayout(top_bar);
  top_layout->setContentsMargins(10, 6, 10, 6);
  const QStringList actions = {
    "New Cell", "Open Scene", "Validate", "Preview", "Generate Scene", "Export"
  };
  for (const QString & label : actions) {
    auto * button = new QPushButton(label, top_bar);
    button->setObjectName("commandBarButton");
    connect(button, &QPushButton::clicked, this, [this, label]() {
      statusBar()->showMessage(label + " action selected.");
      if (label == "Validate" || label == "Generate Scene") {
        ui->error_label->setText("<font color='#2E86C1'>" + label + " action opened from Workcell Studio shell.</font>");
      }
    });
    top_layout->addWidget(button);
  }
  full_screen_button_ = new QPushButton("Full Screen", top_bar);
  connect(full_screen_button_, &QPushButton::clicked, this, &MainWindow::toggle_full_screen);
  top_layout->addWidget(full_screen_button_);
  top_layout->addStretch(1);

  studio_nav_ = new QListWidget(content);
  studio_nav_->addItems({"Dashboard", "Scene Builder", "Assets", "Templates", "Existing Scenes", "Validation", "Export"});
  studio_nav_->setCurrentRow(0);
  studio_nav_->setMaximumWidth(190);

  studio_pages_ = new QStackedWidget(content);
  auto * dashboard_page = new QWidget(studio_pages_);
  auto * dashboard_layout = new QVBoxLayout(dashboard_page);
  auto make_card = [this, dashboard_page](const QString & title, const QString & msg) {
    auto * card = new QPushButton(title, dashboard_page);
    card->setObjectName("studioCardButton");
    connect(card, &QPushButton::clicked, this, [this, msg]() {
      statusBar()->showMessage(msg);
      if (msg.contains("soon")) {
        QMessageBox::information(this, "Workcell Studio", msg);
      }
    });
    return card;
  };
  dashboard_layout->addWidget(new QLabel("<h2>Workcell Studio</h2><p>Dashboard</p>", dashboard_page));
  dashboard_layout->addWidget(make_card("New Workcell", "Use 'Select workspace directory' to start a new workcell."));
  dashboard_layout->addWidget(make_card("Open Existing Scene", "Open Existing Scene from Scene Builder controls."));
  dashboard_layout->addWidget(make_card("Scenario Templates", "Scenario Templates coming soon / not wired yet."));
  dashboard_layout->addWidget(make_card("Asset Browser", "Asset Browser coming soon / not wired yet."));
  dashboard_layout->addWidget(make_card("Validate Current Cell", "Validate action selected."));
  dashboard_layout->addWidget(make_card("Export Demo Bundle", "Export demo bundle coming soon / not wired yet."));
  dashboard_layout->addWidget(new QLabel("Recent / Existing Scenes
- Loaded from selected workspace scenes/", dashboard_page));
  dashboard_layout->addLayout(ui->verticalLayout);

  auto * scene_builder = new QWidget(studio_pages_);
  auto * sb_layout = new QVBoxLayout(scene_builder);
  sb_layout->addWidget(new QLabel("<h2>Scene Builder</h2>", scene_builder));
  auto * tri = new QHBoxLayout();
  tri->addWidget(new QLabel("Scene Hierarchy / Asset Catalog", scene_builder));
  tri->addWidget(new QLabel("3D preview / generated preview will appear here", scene_builder));
  tri->addWidget(new QLabel("Inspector
Selected scene
Robot
Gripper
Transform
Task intent
Readiness
EPD adapter metadata", scene_builder));
  sb_layout->addLayout(tri);
  auto * tabs = new QTabWidget(scene_builder);
  tabs->addTab(new QLabel("Validation results will appear here", scene_builder), "Validation");
  tabs->addTab(new QLabel("Logs output", scene_builder), "Logs");
  tabs->addTab(new QLabel("Launch commands", scene_builder), "Launch Commands");
  tabs->addTab(new QLabel("Readiness dashboard", scene_builder), "Readiness");
  sb_layout->addWidget(tabs);

  studio_pages_->addWidget(dashboard_page);
  studio_pages_->addWidget(scene_builder);
  for (int i = 0; i < 5; ++i) {
    auto * placeholder = new QLabel("Coming soon / not wired yet", studio_pages_);
    placeholder->setAlignment(Qt::AlignCenter);
    studio_pages_->addWidget(placeholder);
  }

  auto * body = new QHBoxLayout();
  body->addWidget(studio_nav_);
  body->addWidget(studio_pages_, 1);

  root_layout->insertWidget(0, top_bar);
  root_layout->insertLayout(1, body, 1);

  connect(studio_nav_, &QListWidget::currentRowChanged, this, [this](int idx) {
    if (idx >= 0 && idx < studio_pages_->count()) {
      studio_pages_->setCurrentIndex(idx);
    }
  });

  auto * esc = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  connect(esc, &QShortcut::activated, this, [this]() {
    if (isFullScreen()) {
      toggle_full_screen();
    }
  });
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
