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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <string>

#include "gui/ui_mainwindow.h"
#include "gui/scene_select.h"
#include "attributes/scene.h"

namespace fs = boost::filesystem;

namespace {
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
  setWindowTitle("Workcell Builder");
  ui->next->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  success = false;
  ui->error_label->setWordWrap(true);
  ui->error_label->setText("<font color='red'>Workcell not available</font>");

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

  for (const auto & supported_distro : ros_dist) {
    ui->ros_distro->addItem(QString::fromStdString(supported_distro));
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
  if (!workcell_file.isEmpty()) {
    fs::current_path(workcell_file.toStdString());
    if (!fs::exists("src")) {
      fs::create_directory("src");
    }
    fs::current_path("src");
    workcell_path = fs::current_path();
    if (!fs::exists("assets")) {
      fs::create_directory("assets");
    }
    if (!fs::exists("scenes")) {
      fs::create_directory("scenes");
    }

    fs::current_path("assets");
    if (!fs::exists("robots")) {
      fs::create_directory("robots");
    }
    if (!fs::exists("end_effectors")) {
      fs::create_directory("end_effectors");
    }
    if (!fs::exists("environment")) {
      fs::create_directory("environment");
    }

    const fs::path assets_path = workcell_path / "assets";
    const fs::path package_assets_path =
      fs::path(ament_index_cpp::get_package_share_directory("workcell_builder")) / "assets";
    const std::array<std::string, 3> asset_subdirs = { "robots", "end_effectors", "environment" };
    for (const auto & asset_subdir : asset_subdirs) {
      const fs::path target_path = assets_path / asset_subdir;
      if (is_empty_directory(target_path)) {
        copy_directory_contents(package_assets_path / asset_subdir, target_path);
      }
    }

    fs::current_path(workcell_path);
    fs::current_path("scenes");

    workcell.scene_vector.clear();
    for (auto & filepath :
      fs::directory_iterator(fs::current_path()))
    {
      Scene temp_scene;
      temp_scene.filepath = filepath.path().string();

      std::string scene_name = filepath.path().filename().string();
      if (is_good_scene(fs::current_path(), scene_name)) {
        temp_scene.name = scene_name;
        temp_scene.loaded = false;
        workcell.scene_vector.push_back(temp_scene);
      }
    }
    ui->error_label->setText("<font color='green'>Workcell loaded</font>");
    ui->next->setDisabled(false);
    ui->load_workcell->setDisabled(true);
    ui->change_workcell->setDisabled(false);
    success = true;
    ui->filepath->setText(workcell_file);
    workcell.workcell_filepath = workcell_file.toStdString();
  }
}

void MainWindow::on_next_clicked()
{
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
  ui->next->setDisabled(true);
  ui->load_workcell->setDisabled(false);
  ui->change_workcell->setDisabled(true);
  ui->filepath->clear();
}
bool MainWindow::is_good_scene(boost::filesystem::path original_path, std::string scene_name)
{
  boost::filesystem::current_path(original_path);
  boost::system::error_code ec;
  if (!boost::filesystem::is_directory(scene_name, ec) || ec)
  {
    return false;
  }
  boost::filesystem::current_path(scene_name);

  if (!boost::filesystem::exists("urdf") || !boost::filesystem::exists("environment.yaml") ||
    !boost::filesystem::exists("CMakeLists.txt") || !boost::filesystem::exists("package.xml"))
  {
    boost::filesystem::current_path(original_path);
    return false;
  }
  boost::filesystem::current_path(original_path);
  return true;
}
