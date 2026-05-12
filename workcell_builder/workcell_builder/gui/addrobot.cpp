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

#include "gui/addrobot.h"
#include "workcell_builder_ui_utils.hpp"


#include <boost/filesystem.hpp>
#include <QKeyEvent>
#include <QMessageBox>
#include <QProcess>
#include <QRegularExpression>
#include <iostream>
#include <fstream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gui/ui_addrobot.h"
#include "include/asset_picker_dialog.hpp"
#include "include/asset_discovery_helper.h"
#include <QPushButton>
#include <QVBoxLayout>

namespace
{
class ScopedCurrentPath
{
public:
  explicit ScopedCurrentPath(const boost::filesystem::path & destination)
  : original_path_(boost::filesystem::current_path()),
    active_(false)
  {
    if (!boost::filesystem::exists(destination) || !boost::filesystem::is_directory(destination)) {
      throw boost::filesystem::filesystem_error(
              "ScopedCurrentPath destination is invalid", destination,
              boost::system::error_code());
    }
    boost::filesystem::current_path(destination);
    active_ = true;
  }

  ~ScopedCurrentPath()
  {
    if (active_) {
      boost::filesystem::current_path(original_path_);
    }
  }

private:
  boost::filesystem::path original_path_;
  bool active_;
};

constexpr const char * kMissingUrdfLabel = "Missing URDF";
const std::unordered_map<std::string, std::vector<std::string>> kDescriptionPackagesByBrand = {
  {"fanuc", {"moveit_resources_fanuc_description", "fanuc_description"}},
  {"panda_robot", {"moveit_resources_panda_description", "panda_description"}}
};

std::string resolve_universal_robot_xacro_filename(
  const boost::filesystem::path & ur_description_path,
  const std::string & robot_name)
{
  if (!ur_description_path.empty()) {
    const boost::filesystem::path model_xacro =
      ur_description_path / "urdf" / (robot_name + ".urdf.xacro");
    const boost::filesystem::path config_dir = ur_description_path / "config" / robot_name;
    if (boost::filesystem::exists(config_dir)) {
      const boost::filesystem::path ur_urdf = ur_description_path / "urdf" / "ur.urdf.xacro";
      if (boost::filesystem::exists(ur_urdf)) {
        return ur_urdf.filename().string();
      }
      if (boost::filesystem::exists(model_xacro)) {
        return model_xacro.filename().string();
      }
      return "ur.urdf.xacro";
    }
    if (boost::filesystem::exists(model_xacro)) {
      return model_xacro.filename().string();
    }
  }
  return robot_name + ".urdf.xacro";
}

boost::filesystem::path resolve_description_package_path(const std::string & brand)
{
  const auto it = kDescriptionPackagesByBrand.find(brand);
  if (it == kDescriptionPackagesByBrand.end()) {
    return {};
  }
  for (const auto & package_name : it->second) {
    try {
      const auto share = ament_index_cpp::get_package_share_directory(package_name);
      if (!share.empty()) {
        return boost::filesystem::path(share);
      }
    } catch (const std::exception & e) {
      std::cerr
        << "Failed to resolve description package share directory for "
        << package_name << ": " << e.what() << std::endl;
    }
  }
  return {};
}

std::vector<boost::filesystem::path> robot_description_candidates(
  const boost::filesystem::path & urdf_path,
  const std::string & robot_name)
{
  return {
    urdf_path / (robot_name + ".urdf"),
    urdf_path / (robot_name + ".urdf.xacro")
  };
}

boost::filesystem::path select_existing_description_file(
  const std::vector<boost::filesystem::path> & candidates)
{
  for (const auto & candidate : candidates) {
    if (boost::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return candidates.empty() ? boost::filesystem::path() : candidates.front();
}
}  // namespace

AddRobot::AddRobot(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddRobot)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
  auto * picker_btn = new QPushButton("Select Robot Asset", this);
  if (layout()) { layout()->addWidget(picker_btn); }
  connect(picker_btn, &QPushButton::clicked, this, [this]() {
    const auto report = discover_workcell_assets(workcell_path.string(), boost::filesystem::current_path().string());
    AssetPickerDialog dialog("Select Robot Asset", this);
    dialog.set_candidates(report.robots, report.robot_paths);
    if (dialog.exec() != QDialog::Accepted) { return; }
    const auto c = dialog.selected_candidate();
    if (c.description_package.empty()) { QMessageBox::warning(this, "Incomplete robot", "Selected robot asset is incomplete: missing description"); return; }
    if (c.moveit_config_package.empty()) { QMessageBox::warning(this, "Incomplete robot", "Selected robot asset is incomplete: missing moveit config"); return; }
    int brand_index = ui->robot_brand->findText(QString::fromStdString("universal_robot"));
    if (brand_index >= 0) ui->robot_brand->setCurrentIndex(brand_index);
    int model_index = ui->robot_model->findText(QString::fromStdString(c.label));
    if (model_index >= 0) ui->robot_model->setCurrentIndex(model_index);
    int base_idx = ui->robot_links->findText("base_link"); if (base_idx >= 0) ui->robot_links->setCurrentIndex(base_idx);
    int ee_idx = ui->robot_ee_link->findText("ee_link"); if (ee_idx >= 0) ui->robot_ee_link->setCurrentIndex(ee_idx);
  });
  on_include_origin_stateChanged(0);
  success = false;
}

AddRobot::~AddRobot()
{
  delete ui;
}
void AddRobot::LoadExistingRobot(Robot robot_input)
{
  robot = robot_input;
  // Populate Origin
  if (robot.origin.is_origin) {
    ui->include_origin->setChecked(true);
    on_include_origin_stateChanged(2);
    if (robot.origin.x >= 0) {ui->x_2->setText(QString::number(robot.origin.x));}
    if (robot.origin.y >= 0) {ui->y_2->setText(QString::number(robot.origin.y));}
    if (robot.origin.z >= 0) {ui->z_2->setText(QString::number(robot.origin.z));}
    if (robot.origin.roll >= 0) {ui->roll_2->setText(QString::number(robot.origin.roll));}
    if (robot.origin.pitch >= 0) {ui->pitch_2->setText(QString::number(robot.origin.pitch));}
    if (robot.origin.yaw >= 0) {ui->yaw_2->setText(QString::number(robot.origin.yaw));}
  } else {
    ui->include_origin->stateChanged(0);
  }

  // Populate Brand
  for (int brand = 0; brand < static_cast<int>(available_brands.size()); brand++) {
    if (robot.brand.compare(available_brands[brand]) == 0) {
      ui->robot_brand->setCurrentIndex(brand);
      for (int model = 0; model < static_cast<int>(available_robots[brand].size()); model++) {
        if (robot.name.compare(available_robots[brand][model].name) == 0) {
          ui->robot_model->setCurrentIndex(model);
          for (int link = 0; link <
            static_cast<int>(available_robots[brand][model].robot_links.size());
            link++)
          {
            if (robot.base_link.compare(available_robots[brand][model].robot_links[link]) == 0) {
              ui->robot_links->setCurrentIndex(link);
            }
            if (robot.ee_link.compare(available_robots[brand][model].robot_links[link]) == 0) {
              ui->robot_ee_link->setCurrentIndex(link);
            }
          }
          break;
        }
      }
      break;
    }
  }
}

int AddRobot::LoadAvailableRobots()
{
  ui->robot_brand->clear();
  ui->robot_model->clear();
  ui->robot_links->clear();
  ui->robot_ee_link->clear();
  available_brands.clear();
  available_robots.clear();
  try {
    const boost::filesystem::path resolved_assets_path = assets_path.empty() ?
      (workcell_path / "assets") : assets_path;
    const boost::filesystem::path robots_path = resolved_assets_path / "robots";
    if (!boost::filesystem::exists(robots_path) || !boost::filesystem::is_directory(robots_path)) {
      QMessageBox::warning(
        this, "Robot Assets Missing",
        QString("Robot assets directory does not exist:\n%1")
        .arg(QString::fromStdString(robots_path.string())));
      return 0;
    }
    if (boost::filesystem::is_empty(robots_path)) {
      return 0;
    }

    for (const auto & filepath : boost::filesystem::directory_iterator(robots_path)) {
      if (!boost::filesystem::is_directory(filepath.path())) {
        continue;
      }
      const boost::filesystem::path brand_path = filepath.path();
      const std::string temp_brand = brand_path.filename().string();
      std::vector<Robot> brand_robot_vector;

      if (temp_brand.compare("universal_robot") == 0) {
        std::vector<std::string> valid_robots;
        for (const auto & model_path : boost::filesystem::directory_iterator(brand_path)) {
          std::string temp_model = model_path.path().filename().string();
          if (temp_model.size() > 14 && temp_model.substr(temp_model.size() - 14) == "_moveit_config") {
            valid_robots.push_back(temp_model.substr(0, temp_model.size() - 14));
          }
        }
        brand_robot_vector = LoadUR(valid_robots);
      } else {
        std::vector<std::string> moveit_configs;
        for (const auto & model_path : boost::filesystem::directory_iterator(brand_path)) {
          std::string temp_model = model_path.path().filename().string();
          if (temp_model.size() > 14 && temp_model.substr(temp_model.size() - 14) == "_moveit_config") {
            moveit_configs.push_back(temp_model.substr(0, temp_model.size() - 14));
          }
        }

        for (const auto & moveit_config : moveit_configs) {
          const std::string description_folder = moveit_config + "_description";
          boost::filesystem::path description_path = brand_path / description_folder;
          if (!boost::filesystem::exists(description_path)) {
            description_path = resolve_description_package_path(temp_brand);
          }
          if (description_path.empty()) {
            std::cerr
              << "No description folder found for " << moveit_config
              << " under assets/robots/" << temp_brand
              << " and no matching ROS 2 description package is installed."
              << std::endl;
            continue;
          }
          brand_robot_vector.push_back(
            LoadRobot(
              description_folder,
              temp_brand,
              description_path));
        }
      }
      if (!brand_robot_vector.empty()) {
        available_robots.push_back(brand_robot_vector);
        available_brands.push_back(temp_brand);
      }
    }
    for (int i3 = 0; i3 < static_cast<int>(available_brands.size()); i3++) {
      ui->robot_brand->addItem(QString::fromStdString(available_brands[i3]));
    }

    ui->robot_brand->setCurrentIndex(0);
    on_robot_brand_currentIndexChanged(0);
    ui->robot_model->setCurrentText(0);
    on_robot_model_currentIndexChanged(0);
    ui->robot_links->setCurrentIndex(0);
    ui->robot_ee_link->setCurrentIndex(0);
    int total_count = 0;
    for (int i = 0; i < static_cast<int>(available_brands.size()); i++) {
      for (int i2 = 0; i2 < static_cast<int>(available_robots[i].size()); i2++) {
        total_count++;
      }
    }
    return total_count;
  } catch (const boost::filesystem::filesystem_error & error) {
    QMessageBox::warning(
      this, "Robot Discovery Error",
      QString("Filesystem error while loading robots:\n%1")
      .arg(QString::fromStdString(error.what())));
  } catch (const std::exception & error) {
    QMessageBox::warning(
      this, "Robot Discovery Error",
      QString("Error while loading robots:\n%1")
      .arg(QString::fromStdString(error.what())));
  }
  return 0;
}

Robot AddRobot::LoadRobot(
  std::string file,
  std::string brand,
  const boost::filesystem::path & description_path)
{
  Robot temp_robot;
  const boost::filesystem::path robots_root = assets_path.empty() ?
    (workcell_path / "assets" / "robots") : (assets_path / "robots");
  const boost::filesystem::path resolved_description_path = description_path.empty() ?
    (robots_root / brand / file) : description_path;
  temp_robot.filepath = resolved_description_path.string();
  temp_robot.brand = brand;
  temp_robot.name = file.erase(file.length() - 12);
  const boost::filesystem::path urdf_path = resolved_description_path / "urdf";
  const auto candidates = robot_description_candidates(urdf_path, temp_robot.name);
  for (const auto & candidate : candidates) {
    if (!boost::filesystem::exists(candidate)) {
      continue;
    }
    temp_robot.robot_links = GetLinks(candidate.string());
    if (!temp_robot.robot_links.empty()) {
      break;
    }
  }
  if (temp_robot.robot_links.empty()) {
    const boost::filesystem::path urdf_file = select_existing_description_file(candidates);
    temp_robot.robot_links = GetLinks(urdf_file.string());
  }
  return temp_robot;
}

std::vector<Robot> AddRobot::LoadUR(std::vector<std::string> robot_list)
// load ur robots based on the way it is structured
{
  const boost::filesystem::path base_path = assets_path.empty() ?
    (workcell_path / "assets" / "robots" / "universal_robot") :
    (assets_path / "robots" / "universal_robot");
  const boost::filesystem::path ur_description_path = base_path / "ur_description";
  boost::filesystem::path resolved_ur_description_path = ur_description_path;
  if (!boost::filesystem::exists(ur_description_path)) {
    std::cerr
      << "ur_description directory not found at expected path: "
      << ur_description_path.string() << std::endl;
    bool resolved_package = false;
    try {
      const auto share = ament_index_cpp::get_package_share_directory("ur_description");
      boost::filesystem::path share_path(share);
      if (boost::filesystem::exists(share_path)) {
        std::cerr
          << "Resolved ur_description via package share directory: "
          << share_path.string() << std::endl;
        resolved_ur_description_path = share_path;
        resolved_package = true;
      }
    } catch (const std::exception & e) {
      std::cerr
        << "Failed to resolve ur_description package share directory: "
        << e.what() << std::endl;
    }
    if (!resolved_package) {
      QMessageBox::warning(
        this,
        "UR Description Missing",
        QString(
          "Unable to locate the ur_description directory.\n"
          "Expected path: %1\n"
          "Please install the ur_description package or fix the assets layout.")
        .arg(QString::fromStdString(ur_description_path.string())));
      return {};
    }
  } else {
    resolved_ur_description_path = ur_description_path;
  }

  std::vector<Robot> ur_robot_vector;
  const boost::filesystem::path urdf_path = resolved_ur_description_path / "urdf";
  if (!boost::filesystem::exists(urdf_path) || !boost::filesystem::is_directory(urdf_path)) {
    QMessageBox::warning(
      this,
      "UR Description Missing",
      QString("URDF folder not found under ur_description:\n%1")
      .arg(QString::fromStdString(urdf_path.string())));
    return {};
  }
  for (int i = 0; i < static_cast<int>(robot_list.size()); i++) {
    Robot temp_robot;
    temp_robot.brand = "universal_robot";
    temp_robot.name = robot_list[i];
    const boost::filesystem::path resolved_xacro =
      urdf_path / resolve_universal_robot_xacro_filename(
      resolved_ur_description_path, temp_robot.name);
    temp_robot.robot_links = GetLinks(
      resolved_xacro.string(),
      {
        "ur_type:=" + temp_robot.name,
        "name:=" + temp_robot.name
      });
    ur_robot_vector.push_back(temp_robot);
  }
  return ur_robot_vector;
}

std::vector<std::string> AddRobot::GetLinks(
  std::string filename,
  const std::vector<std::string> & xacro_arguments)
{
  std::vector<std::string> links;
  boost::filesystem::path file_path;
  try {
    file_path = boost::filesystem::absolute(filename);
  } catch (const boost::filesystem::filesystem_error & error) {
    QMessageBox::warning(
      this,
      "URDF Path Error",
      QString("Failed to resolve URDF/xacro path:\n%1")
      .arg(QString::fromStdString(error.what())));
    return links;
  }
  if (!boost::filesystem::exists(file_path)) {
    std::cerr << "URDF/xacro file not found: " << file_path.string() << std::endl;
    QMessageBox::warning(
      this,
      "URDF Missing",
      QString("URDF/xacro file not found:\n%1")
      .arg(QString::fromStdString(file_path.string())));
    return links;
  }
  std::unique_ptr<ScopedCurrentPath> path_guard;
  try {
    path_guard = std::make_unique<ScopedCurrentPath>(file_path.parent_path());
  } catch (const boost::filesystem::filesystem_error & error) {
    QMessageBox::warning(
      this,
      "URDF Path Error",
      QString("Cannot switch to URDF directory:\n%1")
      .arg(QString::fromStdString(error.what())));
    return links;
  }

  QString urdf_xml;
  QProcess xacro_process;
  QStringList xacro_args;
  xacro_args << QString::fromStdString(file_path.string());
  for (const auto & arg : xacro_arguments) {
    xacro_args << QString::fromStdString(arg);
  }
  xacro_process.start("xacro", xacro_args);
  if (xacro_process.waitForFinished(5000) &&
    xacro_process.exitStatus() == QProcess::NormalExit &&
    xacro_process.exitCode() == 0)
  {
    urdf_xml = QString::fromUtf8(xacro_process.readAllStandardOutput());
  } else {
    const QString stderr_output = QString::fromUtf8(xacro_process.readAllStandardError());
    std::cerr
      << "Failed to expand xacro for file: " << file_path.string()
      << ". Error: " << stderr_output.toStdString() << std::endl;
    QMessageBox::warning(
      this,
      "Xacro Expansion Failed",
      QString(
        "Failed to expand xacro file:\n%1\n\n"
        "Please fix the robot description and try again.")
      .arg(QString::fromStdString(file_path.string())));
    std::ifstream infile(file_path.string());
    if (!infile) {
      std::cerr
        << "Failed to open URDF/xacro file after xacro error: "
        << file_path.string() << std::endl;
      QMessageBox::warning(
        this,
        "URDF Read Failed",
        QString(
          "Failed to open URDF/xacro file:\n%1\n\n"
          "Please check the file path and permissions.")
        .arg(QString::fromStdString(file_path.string())));
      return links;
    }
    std::string content((std::istreambuf_iterator<char>(infile)), std::istreambuf_iterator<char>());
    urdf_xml = QString::fromStdString(content);
  }

  QRegularExpression link_regex(R"(<link\s+[^>]*name\s*=\s*\"([^\"]+)\")");
  QRegularExpressionMatchIterator matches = link_regex.globalMatch(urdf_xml);
  std::set<std::string> seen_links;
  while (matches.hasNext()) {
    QRegularExpressionMatch match = matches.next();
    std::string link_name = match.captured(1).toStdString();
    if (seen_links.insert(link_name).second) {
      links.push_back(link_name);
    }
  }
  return links;
}

void AddRobot::on_robot_brand_currentIndexChanged(int index)
{
  bool oldState = ui->robot_model->blockSignals(true);
  ui->robot_model->clear();
  for (int i = 0; i < static_cast<int>(available_robots[index].size()); i++) {
    ui->robot_model->addItem(QString::fromStdString(available_robots[index][i].name));
  }
  ui->robot_model->blockSignals(oldState);
  ui->robot_model->setCurrentIndex(0);
  on_robot_model_currentIndexChanged(0);
}


void AddRobot::on_robot_model_currentIndexChanged(int index)
{
  Q_UNUSED(index);
  bool oldState = ui->robot_links->blockSignals(true);
  bool oldState2 = ui->robot_ee_link->blockSignals(true);
  ui->robot_links->clear();
  ui->robot_ee_link->clear();
  const auto & selected_links =
    available_robots[ui->robot_brand->currentIndex()][ui->robot_model->currentIndex()]
    .robot_links;
  if (selected_links.empty()) {
    ui->robot_links->addItem(kMissingUrdfLabel);
    ui->robot_ee_link->addItem(kMissingUrdfLabel);
  }
  for (const auto & link_name : selected_links) {
    ui->robot_links->addItem(QString::fromStdString(link_name));
    ui->robot_ee_link->addItem(QString::fromStdString(link_name));
  }
  ui->robot_links->blockSignals(oldState);
  ui->robot_links->setCurrentIndex(0);
  ui->robot_ee_link->blockSignals(oldState2);
  ui->robot_ee_link->setCurrentIndex(0);
}


void AddRobot::on_include_origin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    // No Origin
    ui->x_2->setDisabled(true);
    ui->x_2->setText(QString::fromStdString("0"));
    ui->x_label_2->setDisabled(true);
    ui->y_2->setDisabled(true);
    ui->y_2->setText(QString::fromStdString("0"));
    ui->y_label_2->setDisabled(true);
    ui->z_2->setDisabled(true);
    ui->z_2->setText(QString::fromStdString("0"));
    ui->z_label_2->setDisabled(true);
    ui->roll_2->setDisabled(true);
    ui->roll_2->setText(QString::fromStdString("0"));
    ui->roll_label_2->setDisabled(true);
    ui->pitch_2->setDisabled(true);
    ui->pitch_2->setText(QString::fromStdString("0"));
    ui->pitch_label_2->setDisabled(true);
    ui->yaw_2->setDisabled(true);
    ui->yaw_2->setText(QString::fromStdString("0"));
    ui->yaw_label_2->setDisabled(true);
    ui->origin_label_2->setDisabled(true);
    ui->position_label_2->setDisabled(true);
    ui->orientation_label_2->setDisabled(true);
  } else {
    // Have origin
    ui->x_2->setDisabled(false);
    ui->x_2->clear();
    ui->x_label_2->setDisabled(false);
    ui->y_2->setDisabled(false);
    ui->y_2->clear();
    ui->y_label_2->setDisabled(false);
    ui->z_2->setDisabled(false);
    ui->z_2->clear();
    ui->z_label_2->setDisabled(false);
    ui->roll_2->setDisabled(false);
    ui->roll_2->clear();
    ui->roll_label_2->setDisabled(false);
    ui->pitch_2->setDisabled(false);
    ui->pitch_2->clear();
    ui->pitch_label_2->setDisabled(false);
    ui->yaw_2->setDisabled(false);
    ui->yaw_2->clear();
    ui->yaw_label_2->setDisabled(false);
    ui->origin_label_2->setDisabled(false);
    ui->position_label_2->setDisabled(false);
    ui->orientation_label_2->setDisabled(false);
  }
}

void AddRobot::on_exit_2_clicked()
{
  success = false;
  this->close();
}

void AddRobot::on_ok_2_clicked()
{
  if (ui->robot_brand->currentIndex() < 0 || ui->robot_model->currentIndex() < 0) {
    ui->errorlist_2->append(" <font color='red'> Invalid Robot Brand or Robot Model \n </font>");
  } else {
    robot = available_robots[ui->robot_brand->currentIndex()][ui->robot_model->currentIndex()];
    ui->errorlist_2->clear();
    if (robot.robot_links.empty()) {
      ui->errorlist_2->append(
        " <font color='red'> Missing URDF/xacro for selected robot."
        " Please fix the robot description before continuing. \n </font>");
      return;
    }
    if (ErrorCheckOrigin() == 0) {
      if (ui->robot_links->currentIndex() < 0) {
        ui->errorlist_2->append(" <font color='red'> Invalid Base Link \n </font>");
        return;
      }
      if (ui->robot_ee_link->currentIndex() < 0) {
        ui->errorlist_2->append(" <font color='red'> Invalid EE Link \n </font>");
        return;
      }
      robot.base_link = ui->robot_links->currentText().toStdString();
      robot.ee_link = ui->robot_ee_link->currentText().toStdString();
      success = true;
      this->close();
    }
  }
}

int AddRobot::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (ui->include_origin->isChecked()) {
    robot.origin.is_origin = true;
    bool all_empty_xyz =
      (ui->x_2->text().isEmpty() && ui->y_2->text().isEmpty() && ui->z_2->text().isEmpty());
    bool all_full_xyz =
      (!ui->x_2->text().isEmpty() && !ui->y_2->text().isEmpty() && !ui->z_2->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll_2->text().isEmpty() && ui->pitch_2->text().isEmpty() &&
      ui->yaw_2->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll_2->text().isEmpty() && !ui->pitch_2->text().isEmpty() &&
      !ui->yaw_2->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist_2->append(
        " <font color='red'> All Origin Fields are empty."
        " Uncheck the Origin selection to disable the option. \n </font>");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist_2->append(
          "<font color='red'> XYZ values not complete."
          " Leave it all blank for default values. \n </font>");
        num_errors++;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->x_2->text();
          QString input_y = ui->y_2->text();
          QString input_z = ui->z_2->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist_2->append(
              "<font color='red'> Type Error: XYZ need to be floats \n </font>");
            num_errors++;
          } else {
            robot.origin.x = input_x.toFloat();
            robot.origin.y = input_y.toFloat();
            robot.origin.z = input_z.toFloat();
          }
        }
      }
      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist_2->append(
          "<font color='red'> RPY values not complete."
          " Leave it all blank for default values </font>");
        num_errors++;
      } else {
        if (all_full_rpy) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_roll = ui->roll_2->text();
          QString input_pitch = ui->pitch_2->text();
          QString input_yaw = ui->yaw_2->text();
          if (float_validator->validate(
              input_roll,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_pitch,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_yaw, i) != QValidator::Acceptable)
          {
            ui->errorlist_2->append(
              " <font color='red'> Type Error: RPY need"
              " to be floats </font>");
            num_errors++;
          } else {
            robot.origin.roll = ui->roll_2->text().toFloat();
            robot.origin.pitch = ui->pitch_2->text().toFloat();
            robot.origin.yaw = ui->yaw_2->text().toFloat();
          }
        }
      }
    }
  } else {
    robot.origin.disableOrigin();
  }
  return num_errors;
}

void AddRobot::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
