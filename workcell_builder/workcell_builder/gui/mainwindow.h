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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_

#include <QFutureWatcher>
#include <QStackedWidget>
#include <QMainWindow>
#include <QTableWidget>
#include <QLabel>
#include <QString>
#include <atomic>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include "attributes/workcell.h"
#include "workcell_studio_scene_browser.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {class MainWindow;}
QT_END_NAMESPACE
class QProgressDialog;
class QListWidget;
class QPushButton;
class QTextEdit;

class MainWindow: public QMainWindow
{
  Q_OBJECT

public:
  friend class TestMain;
  boost::filesystem::path workcell_path;
  Workcell workcell;
  bool success;
  // Supported ROS 2 distributions detected at runtime.
  std::vector<std::string> ros_dist;
  bool is_good_scene(boost::filesystem::path original_path, std::string scene_name);

  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

private slots:
  void on_load_workcell_clicked();

  void on_next_clicked();

  void on_change_workcell_clicked();

private:
  bool has_selected_ros_distro() const;
  void update_next_button_state();
  void toggle_full_screen();
  void setup_studio_shell();
  void apply_studio_theme();
  void append_studio_log(const QString & message);
  void show_not_wired_message(const QString & action_label);
  void refresh_scene_browser_ui();
  void select_scene_by_row(int row);
  void open_selected_scene_artifact(const QString & artifact);
  QString selected_scene_launch_command() const;
  Ui::MainWindow * ui;
  QStackedWidget * studio_pages_{ nullptr };
  QListWidget * studio_nav_{ nullptr };
  QPushButton * full_screen_button_{ nullptr };
  QTextEdit * studio_log_{ nullptr };
  QTableWidget * dashboard_scene_table_{ nullptr };
  QTableWidget * existing_scene_table_{ nullptr };
  QLabel * dashboard_summary_label_{ nullptr };
  QLabel * scene_builder_title_{ nullptr };
  QLabel * scene_preview_label_{ nullptr };
  QLabel * inspector_label_{ nullptr };
  QLabel * readiness_label_{ nullptr };
  workcell_builder::WorkcellStudioSceneBrowserResult scene_browser_result_;
  int selected_scene_index_{ -1 };
  struct WorkcellLoadResult
  {
    bool success{ false };
    bool cancelled{ false };
    QString error;
    Workcell workcell;
    boost::filesystem::path workcell_path;
    QString workcell_file;
    QString workcell_root_label;
  };
  QFutureWatcher<WorkcellLoadResult> * load_watcher_{ nullptr };
  QProgressDialog * progress_dialog_{ nullptr };
  std::atomic<bool> cancel_requested_{ false };
};
#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__MAINWINDOW_H_
