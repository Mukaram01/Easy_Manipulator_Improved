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

#include "gui/addendeffector.h"
#include <QString>
#include <QKeyEvent>
#include <QMessageBox>
#include <QProcess>
#include <QRegularExpression>
#include <QTemporaryFile>
#include <QTextStream>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gui/ui_addendeffector.h"
#include "attributes/robot.h"

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
}  // namespace

AddEndEffector::AddEndEffector(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddEndEffector)
{
  ui->setupUi(this);
  for (int i = 0; i < static_cast<int>(supported_types.size()); i++) {
    ui->ee_type->addItem(QString::fromStdString(supported_types[i]));
  }
  on_include_origin_stateChanged(0);
  success = false;
}


int AddEndEffector::LoadAvailableEE(Robot robot, const boost::filesystem::path & end_effector_path)
{
  ui->parent_object->setText(QString::fromStdString(robot.name));
  ui->parent_link->setText(QString::fromStdString(robot.ee_link));
  const boost::filesystem::path resolved_assets_path = assets_path.empty() ?
    (workcell_path / "assets") : assets_path;
  boost::filesystem::path ee_path = end_effector_path.empty() ?
    (resolved_assets_path / "end_effectors") : end_effector_path;
  ui->ee_brand->clear();
  ui->ee_model->clear();
  ui->ee_links->clear();
  available_brands.clear();
  available_ee.clear();
  try {
    if (!boost::filesystem::exists(ee_path) || boost::filesystem::is_empty(ee_path)) {
      const auto share =
        ament_index_cpp::get_package_share_directory("workcell_builder");
      ee_path = boost::filesystem::path(share) / "assets" / "end_effectors";
    }
    if (!boost::filesystem::exists(ee_path) || boost::filesystem::is_empty(ee_path)) {
      return 0;
    }

    for (const auto & filepath : boost::filesystem::directory_iterator(ee_path)) {
      if (!boost::filesystem::is_directory(filepath.path())) {
        continue;
      }
      const boost::filesystem::path brand_path = filepath.path();
      const std::string temp_brand = brand_path.filename().string();
      std::vector<EndEffector> brand_ee_vector;

      std::vector<std::string> moveit_configs;
      std::vector<std::string> descriptions;
      for (const auto & filepath_model : boost::filesystem::directory_iterator(brand_path)) {
        const std::string temp_model = filepath_model.path().filename().string();
        if (temp_model.size() > 12 && temp_model.substr(temp_model.size() - 12) == "_description") {
          descriptions.push_back(temp_model.substr(0, temp_model.size() - 12));
        }
        if (temp_model.size() > 14 && temp_model.substr(temp_model.size() - 14) == "_moveit_config") {
          moveit_configs.push_back(temp_model.substr(0, temp_model.size() - 14));
        }
      }

      if (moveit_configs.size() < descriptions.size()) {
        for (int i = 0; i < static_cast<int>(descriptions.size()); i++) {
          if (std::find(
              moveit_configs.begin(), moveit_configs.end(),
              descriptions[i]) != moveit_configs.end())
          {
            brand_ee_vector.push_back(
              LoadEE((brand_path / (descriptions[i] + "_description")), temp_brand));
          }
        }
      } else {
        for (int i = 0; i < static_cast<int>(moveit_configs.size()); i++) {
          if (std::find(
              descriptions.begin(), descriptions.end(),
              moveit_configs[i]) != descriptions.end())
          {
            brand_ee_vector.push_back(
              LoadEE((brand_path / (moveit_configs[i] + "_description")), temp_brand));
          }
        }
      }

      if (!brand_ee_vector.empty()) {
        available_ee.push_back(brand_ee_vector);
        available_brands.push_back(temp_brand);
      }
    }
    int total_ee = 0;
    for (int i = 0; i < static_cast<int>(available_brands.size()); i++) {
      for (int j = 0; j < static_cast<int>(available_ee[i].size()); j++) {
        total_ee++;
      }
    }
    if (total_ee == 0) {
      return 0;
    }
    for (int i3 = 0; i3 < static_cast<int>(available_brands.size()); i3++) {
      ui->ee_brand->addItem(QString::fromStdString(available_brands[i3]));
    }
    ui->ee_brand->setCurrentIndex(0);
    on_ee_brand_currentIndexChanged(0);
    ui->ee_model->setCurrentIndex(0);
    on_ee_model_currentIndexChanged(0);
    ui->ee_links->setCurrentIndex(0);
    return total_ee;
  } catch (const boost::filesystem::filesystem_error & error) {
    QMessageBox::warning(
      this,
      "End Effector Discovery Error",
      QString("Filesystem error while loading end effectors:\n%1")
      .arg(QString::fromStdString(error.what())));
  } catch (const std::exception & error) {
    QMessageBox::warning(
      this,
      "End Effector Discovery Error",
      QString("Error while loading end effectors:\n%1")
      .arg(QString::fromStdString(error.what())));
  }
  return 0;
}

void AddEndEffector::LoadExistingEE(EndEffector ee_input)
{
  ee = ee_input;

  //  Populate Origin
  if (ee.origin.is_origin) {
    ui->include_origin->setChecked(true);
    on_include_origin_stateChanged(2);
    if (ee.origin.x >= 0) {ui->x->setText(QString::number(ee.origin.x));}
    if (ee.origin.y >= 0) {ui->y->setText(QString::number(ee.origin.y));}
    if (ee.origin.z >= 0) {ui->z->setText(QString::number(ee.origin.z));}
    if (ee.origin.roll >= 0) {ui->roll->setText(QString::number(ee.origin.roll));}
    if (ee.origin.pitch >= 0) {ui->pitch->setText(QString::number(ee.origin.pitch));}
    if (ee.origin.yaw >= 0) {ui->yaw->setText(QString::number(ee.origin.yaw));}
  } else {
    ui->include_origin->stateChanged(0);
  }

  //  Populate Brand
  for (int brand = 0; brand < static_cast<int>(available_brands.size()); brand++) {
    if (ee.brand.compare(available_brands[brand]) == 0) {
      ui->ee_brand->setCurrentIndex(brand);
      for (int model = 0; model < static_cast<int>(available_ee[brand].size()); model++) {
        if (ee.name.compare(available_ee[brand][model].name) == 0) {
          ui->ee_model->setCurrentIndex(model);
          for (int link = 0;
            link < static_cast<int>(available_ee[brand][model].ee_links.size()); link++)
          {
            if (ee.base_link.compare(available_ee[brand][model].ee_links[link]) == 0) {
              ui->ee_links->setCurrentIndex(link);
              break;
            }
          }
          break;
        }
      }
      break;
    }
  }
}

void AddEndEffector::on_ee_brand_currentIndexChanged(int index)
{
  bool oldState = ui->ee_model->blockSignals(true);
  ui->ee_model->clear();
  for (int i = 0; i < static_cast<int>(available_ee[index].size()); i++) {
    ui->ee_model->addItem(QString::fromStdString(available_ee[index][i].name));
  }
  ui->ee_model->blockSignals(oldState);
  ui->ee_model->setCurrentIndex(0);
  on_ee_model_currentIndexChanged(0);
}

void AddEndEffector::on_ee_model_currentIndexChanged(int index)
{
  bool oldState = ui->ee_links->blockSignals(true);
  ui->ee_links->clear();
  if (ui->ee_model->currentIndex() >= 0 && ui->ee_brand->currentIndex() >= 0) {
    for (int i = 0;
      i <
      static_cast<int>(available_ee[ui->ee_brand->
      currentIndex()][ui->ee_model->currentIndex()].ee_links.size());
      i++)
    {
      ui->ee_links->addItem(
        QString::fromStdString(
          available_ee[ui->ee_brand->currentIndex()][index]
          .ee_links[i]));
    }
  }
  ui->ee_links->blockSignals(oldState);
}

void AddEndEffector::on_ee_type_currentIndexChanged(int index)
{
  bool oldState1 = ui->attribute_1->blockSignals(true);
  bool oldState2 = ui->attribute_2->blockSignals(true);
  ui->attribute_1->clear();
  ui->attribute_2->clear();
  if (supported_attributes[index].size() == 1) {
    ui->attribute_1_label->setText("Fingers:");
    for (int i = 0; i < static_cast<int>(supported_attributes[index][0].size()); i++) {
      ui->attribute_1->addItem(QString::number(supported_attributes[index][0][i]));
    }
    ui->attribute_2->hide();
    ui->attribute_2_label->hide();
  } else if (supported_attributes[index].size() == 2) {
    ui->attribute_2->show();
    ui->attribute_2_label->show();
    ui->attribute_1_label->setText("Suction array Width");
    ui->attribute_2_label->setText("Suction array Height");
    for (int i = 0; i < static_cast<int>(supported_attributes[index][0].size()); i++) {
      ui->attribute_1->addItem(QString::number(supported_attributes[index][0][i]));
    }
    for (int i = 0; i < static_cast<int>(supported_attributes[index][1].size()); i++) {
      ui->attribute_2->addItem(QString::number(supported_attributes[index][1][i]));
    }
  }
  ui->attribute_1->blockSignals(oldState1);
  ui->attribute_2->blockSignals(oldState2);
}

EndEffector AddEndEffector::LoadEE(const boost::filesystem::path & description_path, std::string brand)
{
  EndEffector temp_ee;
  temp_ee.filepath = description_path.string();
  temp_ee.brand = brand;
  temp_ee.name = description_path.filename().string();
  if (temp_ee.name.size() > 12 && temp_ee.name.substr(temp_ee.name.size() - 12) == "_description") {
    temp_ee.name.erase(temp_ee.name.length() - 12);
  }
  const boost::filesystem::path urdf_dir = description_path / "urdf";
  const std::string macro_filename = temp_ee.name + "_gripper.urdf.xacro";
  const std::string standalone_filename = temp_ee.name + "_gripper_standalone.urdf.xacro";
  if (boost::filesystem::exists(urdf_dir / standalone_filename)) {
    temp_ee.ee_links = GetLinks((urdf_dir / standalone_filename).string());
  } else {
    temp_ee.ee_links = GetLinks((urdf_dir / macro_filename).string());
  }
  return temp_ee;
}
namespace
{
bool HasXacroArgument(const std::vector<std::string> & args, const std::string & key)
{
  for (const auto & arg : args) {
    if (arg.rfind(key, 0) == 0) {
      return true;
    }
  }
  return false;
}

void AppendDefaultXacroArguments(
  const std::string & filename,
  std::vector<std::string> & xacro_arguments)
{
  if (filename.find("robotiq_85_gripper") != std::string::npos) {
    if (!HasXacroArgument(xacro_arguments, "prefix:=")) {
      xacro_arguments.emplace_back("prefix:=");
    }
    if (!HasXacroArgument(xacro_arguments, "parent:=")) {
      xacro_arguments.emplace_back("parent:=world");
    }
  }
}
}  // namespace

std::vector<std::string> AddEndEffector::GetLinks(
  std::string filename,
  const std::vector<std::string> & xacro_arguments)
{
  std::vector<std::string> links;
  const boost::filesystem::path file_path = boost::filesystem::absolute(filename);
  if (!boost::filesystem::exists(file_path)) {
    ui->errorlist->append(
      QString::fromStdString(
        "<font color='red'> End effector xacro file not found: " + file_path.string() + " </font>"));
    return links;
  }
  std::unique_ptr<ScopedCurrentPath> path_guard;
  try {
    path_guard = std::make_unique<ScopedCurrentPath>(file_path.parent_path());
  } catch (const boost::filesystem::filesystem_error & error) {
    ui->errorlist->append(
      QString::fromStdString(
        "<font color='red'> Cannot open end-effector directory: " + std::string(error.what()) +
        " </font>"));
    return links;
  }

  QString urdf_xml;
  std::vector<std::string> expanded_xacro_arguments = xacro_arguments;
  AppendDefaultXacroArguments(filename, expanded_xacro_arguments);
  auto build_xacro_args = [&](const QString & target_file) {
    QStringList xacro_args;
    xacro_args << target_file;
    for (const auto & arg : expanded_xacro_arguments) {
      xacro_args << QString::fromStdString(arg);
    }
    return xacro_args;
  };

  auto run_xacro = [&](const QStringList & xacro_args, QString * output, QString * stderr_output) {
    QProcess xacro_process;
    xacro_process.start("xacro", xacro_args);
    if (xacro_process.waitForFinished(5000) &&
      xacro_process.exitStatus() == QProcess::NormalExit &&
      xacro_process.exitCode() == 0)
    {
      *output = QString::fromUtf8(xacro_process.readAllStandardOutput());
      return true;
    }
    *stderr_output = QString::fromUtf8(xacro_process.readAllStandardError());
    return false;
  };

  const QString xacro_filename = QString::fromStdString(file_path.string());
  QString xacro_stderr;
  if (!run_xacro(build_xacro_args(xacro_filename), &urdf_xml, &xacro_stderr)) {
    QString error_message =
      QString::fromStdString("<font color='red'> Xacro failed for: " + filename + " </font>");
    if (!xacro_stderr.trimmed().isEmpty()) {
      error_message += QString::fromStdString("<br/><pre>") + xacro_stderr + "</pre>";
    }
    ui->errorlist->append(error_message);
    std::ifstream infile(filename);
    std::string content((std::istreambuf_iterator<char>(infile)), std::istreambuf_iterator<char>());
    urdf_xml = QString::fromStdString(content);
  }

  QRegularExpression link_regex(R"(<link\s+[^>]*name\s*=\s*\"([^\"]+)\")");
  auto extract_links = [&](const QString & xml) {
    std::vector<std::string> extracted_links;
    QRegularExpressionMatchIterator matches = link_regex.globalMatch(xml);
    std::set<std::string> seen_links;
    while (matches.hasNext()) {
      QRegularExpressionMatch match = matches.next();
      std::string link_name = match.captured(1).toStdString();
      if (seen_links.insert(link_name).second) {
        extracted_links.push_back(link_name);
      }
    }
    return extracted_links;
  };

  links = extract_links(urdf_xml);
  if (links.empty()) {
    const QString standalone_suffix = "_standalone.urdf.xacro";
    if (xacro_filename.endsWith(".urdf.xacro") && !xacro_filename.endsWith(standalone_suffix)) {
      QString standalone_candidate = xacro_filename;
      standalone_candidate.replace(".urdf.xacro", standalone_suffix);
      if (boost::filesystem::exists(standalone_candidate.toStdString())) {
        return GetLinks(standalone_candidate.toStdString(), xacro_arguments);
      }
    }

    std::ifstream infile(filename);
    std::string content((std::istreambuf_iterator<char>(infile)), std::istreambuf_iterator<char>());
    QString file_contents = QString::fromStdString(content);
    QRegularExpression macro_regex(R"(<xacro:macro\s+[^>]*name\s*=\s*\"([^\"]+)\")");
    QRegularExpressionMatch macro_match = macro_regex.match(file_contents);
    if (macro_match.hasMatch()) {
      QString macro_name = macro_match.captured(1);
      QRegularExpression macro_tag_regex(R"(<xacro:macro[^>]*>)");
      QRegularExpressionMatch tag_match = macro_tag_regex.match(file_contents, macro_match.capturedStart());
      QString macro_tag = tag_match.hasMatch() ? tag_match.captured(0) : QString();
      QRegularExpression params_regex(R"(params\s*=\s*\"([^\"]*)\")");
      QRegularExpressionMatch params_match = params_regex.match(macro_tag);
      QString params = params_match.hasMatch() ? params_match.captured(1) : QString();
      QStringList param_tokens =
        params.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
      std::set<QString> param_names;
      bool needs_origin_block = false;
      for (const auto & token : param_tokens) {
        if (token.startsWith("*")) {
          QString name = token.mid(1);
          if (name == "origin") {
            needs_origin_block = true;
          }
          if (!name.isEmpty()) {
            param_names.insert(name);
          }
        } else {
          param_names.insert(token);
        }
      }

      QString wrapper_xml = "<?xml version=\"1.0\"?>\n";
      wrapper_xml += "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"ee_wrapper\">\n";
      wrapper_xml += QString("  <xacro:include filename=\"%1\"/>\n").arg(xacro_filename);
      wrapper_xml += QString("  <xacro:%1").arg(macro_name);
      std::set<QString> provided_params;
      for (const auto & arg : expanded_xacro_arguments) {
        const auto delimiter_pos = arg.find(":=");
        if (delimiter_pos == std::string::npos) {
          continue;
        }
        const QString key = QString::fromStdString(arg.substr(0, delimiter_pos));
        const QString value = QString::fromStdString(arg.substr(delimiter_pos + 2));
        if (!param_names.empty() && param_names.find(key) == param_names.end()) {
          continue;
        }
        provided_params.insert(key);
        wrapper_xml += QString(" %1=\"%2\"").arg(key, value);
      }
      if (
        param_names.find("prefix") != param_names.end() &&
        provided_params.find("prefix") == provided_params.end())
      {
        wrapper_xml += " prefix=\"\"";
      }
      if (
        param_names.find("parent") != param_names.end() &&
        provided_params.find("parent") == provided_params.end())
      {
        wrapper_xml += " parent=\"tool0\"";
      }
      if (needs_origin_block) {
        wrapper_xml += ">\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        wrapper_xml += QString("  </xacro:%1>\n").arg(macro_name);
      } else {
        wrapper_xml += "/>\n";
      }
      wrapper_xml += "</robot>\n";

      QTemporaryFile wrapper_file;
      if (wrapper_file.open()) {
        QTextStream wrapper_stream(&wrapper_file);
        wrapper_stream << wrapper_xml;
        wrapper_stream.flush();
        QString wrapper_output;
        QString wrapper_error;
        if (run_xacro(build_xacro_args(wrapper_file.fileName()), &wrapper_output, &wrapper_error)) {
          links = extract_links(wrapper_output);
        } else {
          QString error_message =
            QString::fromStdString("<font color='red'> Xacro wrapper failed for: " + filename +
            " </font>");
          if (!wrapper_error.trimmed().isEmpty()) {
            error_message += QString::fromStdString("<br/><pre>") + wrapper_error + "</pre>";
          }
          ui->errorlist->append(error_message);
        }
      }
    }
  }

  if (links.empty()) {
    ui->errorlist->append(
      "<font color='red'> No links found; end-effector xacro defines only macros. "
      "Provide a *_standalone.urdf.xacro or macro instantiation. </font>");
  }
  return links;
}

void AddEndEffector::on_include_origin_stateChanged(int arg1)
{
  if (arg1 == 0) {
    //  No Origin
    ui->x->setDisabled(true);
    ui->x->setText(QString::fromStdString("0"));
    ui->x_label->setDisabled(true);
    ui->y->setDisabled(true);
    ui->y->setText(QString::fromStdString("0"));
    ui->y_label->setDisabled(true);
    ui->z->setDisabled(true);
    ui->z->setText(QString::fromStdString("0"));
    ui->z_label->setDisabled(true);
    ui->roll->setDisabled(true);
    ui->roll->setText(QString::fromStdString("0"));
    ui->roll_label->setDisabled(true);
    ui->pitch->setDisabled(true);
    ui->pitch->setText(QString::fromStdString("0"));
    ui->pitch_label->setDisabled(true);
    ui->yaw->setDisabled(true);
    ui->yaw->setText(QString::fromStdString("0"));
    ui->yaw_label->setDisabled(true);
    ui->origin_label->setDisabled(true);
    ui->position_label->setDisabled(true);
    ui->orientation_label->setDisabled(true);
  } else {
    //  Have origin
    ui->x->setDisabled(false);
    ui->x->clear();
    ui->x_label->setDisabled(false);
    ui->y->setDisabled(false);
    ui->y->clear();
    ui->y_label->setDisabled(false);
    ui->z->setDisabled(false);
    ui->z->clear();
    ui->z_label->setDisabled(false);
    ui->roll->setDisabled(false);
    ui->roll->clear();
    ui->roll_label->setDisabled(false);
    ui->pitch->setDisabled(false);
    ui->pitch->clear();
    ui->pitch_label->setDisabled(false);
    ui->yaw->setDisabled(false);
    ui->yaw->clear();
    ui->yaw_label->setDisabled(false);
    ui->origin_label->setDisabled(false);
    ui->position_label->setDisabled(false);
    ui->orientation_label->setDisabled(false);
  }
}

int AddEndEffector::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (ui->include_origin->isChecked()) {
    ee.origin.is_origin = true;
    bool all_empty_xyz =
      (ui->x->text().isEmpty() && ui->y->text().isEmpty() && ui->z->text().isEmpty());
    bool all_full_xyz =
      (!ui->x->text().isEmpty() && !ui->y->text().isEmpty() && !ui->z->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll->text().isEmpty() && ui->pitch->text().isEmpty() && ui->yaw->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll->text().isEmpty() && !ui->pitch->text().isEmpty() && !ui->yaw->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist->
      append(
        "<font color='red'> All Origin Fields are empty."
        "Uncheck the Origin selection to disable the option. \n </font>");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->
        append(
          "<font color='red'>XYZ values not complete."
          " Leave it all blank for default values. \n </font>");
        num_errors++;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->x->text();
          QString input_y = ui->y->text();
          QString input_z = ui->z->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist->
            append("<font color='red'> Type Error: XYZ need to be floats \n </font>");
            num_errors++;
          } else {
            ee.origin.x = input_x.toFloat();
            ee.origin.y = input_y.toFloat();
            ee.origin.z = input_z.toFloat();
          }
        }
      }
      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist->
        append(
          "<font color='red'> RPY values not complete."
          " Leave it all blank for default values </font>");
        num_errors++;
      } else {
        if (all_full_rpy) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_roll = ui->roll->text();
          QString input_pitch = ui->pitch->text();
          QString input_yaw = ui->yaw->text();
          if (float_validator->validate(
              input_roll,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_pitch,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_yaw, i) != QValidator::Acceptable)
          {
            ui->errorlist->append(" <font color='red'> Type Error: RPY need to be floats </font>");
            num_errors++;
          } else {
            ee.origin.roll = ui->roll->text().toFloat();
            ee.origin.pitch = ui->pitch->text().toFloat();
            ee.origin.yaw = ui->yaw->text().toFloat();
          }
        }
      }
    }
  } else {
    ee.origin.disableOrigin();
  }
  return num_errors;
}

AddEndEffector::~AddEndEffector()
{
  delete ui;
}

void AddEndEffector::on_ok_clicked()
{
  if (ui->ee_brand->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE Brand Error"));
    return;
  }
  if (ui->ee_model->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE model Error"));
    return;
  }
  if (ui->ee_links->currentIndex() < 0) {
    ui->errorlist->append(QString::fromStdString("EE Link Error"));
    return;
  }

  ee = available_ee[ui->ee_brand->currentIndex()][ui->ee_model->currentIndex()];
  ui->errorlist->clear();
  if (ErrorCheckOrigin() == 0) {
    ee.ee_type = ui->ee_type->currentText().toStdString();
    ee.attribute_1 = ui->attribute_1->currentText().toInt();

    if (ee.ee_type.compare("suction")) {
      ee.attribute_2 = ui->attribute_2->currentText().toInt();
    }
    ee.base_link = ui->ee_links->currentText().toStdString();
    ee.robot_link = ui->parent_link->text().toStdString();
    success = true;
    this->close();
  }
}

void AddEndEffector::on_exit_clicked()
{
  success = false;
  this->close();
}

void AddEndEffector::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
