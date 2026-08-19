#include "gui/startup_dialog.h"
#include "include/workspace_validation.hpp"

#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>
#include <QHBoxLayout>

StartupDialog::StartupDialog(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("Workcell Studio Setup");
  setMinimumSize(520, 260);

  auto * root = new QVBoxLayout(this);
  auto * title = new QLabel("<h2>Workcell Studio Setup</h2>", this);
  auto * subtitle = new QLabel("Select your ROS 2 distro and workspace to begin.", this);
  root->addWidget(title);
  root->addWidget(subtitle);

  ros_distro_combo_ = new QComboBox(this);
  ros_distro_combo_->addItem("Humble", "humble");
  ros_distro_combo_->addItem("Foxy (legacy)", "foxy");
  ros_distro_combo_->setItemData(1, 0, Qt::UserRole - 1);
  ros_distro_combo_->addItem("Iron (preview)", "iron");
  ros_distro_combo_->setItemData(2, 0, Qt::UserRole - 1);
  ros_distro_combo_->addItem("Jazzy (preview)", "jazzy");
  ros_distro_combo_->setItemData(3, 0, Qt::UserRole - 1);
  root->addWidget(new QLabel("ROS 2 distro", this));
  root->addWidget(ros_distro_combo_);

  workspace_edit_ = new QLineEdit(this);
  auto * workspace_row = new QHBoxLayout();
  workspace_row->addWidget(workspace_edit_);
  auto * browse = new QPushButton("Browse...", this);
  workspace_row->addWidget(browse);
  root->addWidget(new QLabel("Workspace folder", this));
  root->addLayout(workspace_row);

  status_label_ = new QLabel(this);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  auto * actions = new QHBoxLayout();
  actions->addStretch();
  open_studio_button_ = new QPushButton("Open Studio", this);
  auto * exit_button = new QPushButton("Exit", this);
  actions->addWidget(open_studio_button_);
  actions->addWidget(exit_button);
  root->addLayout(actions);

  QSettings settings("easy_manipulation_deployment", "workcell_builder");
  const QString saved_workspace = settings.value("startup/last_workspace").toString();
  const QString saved_distro = settings.value("startup/last_ros_distro", "humble").toString();

  const QString ws = saved_workspace.isEmpty() ? default_workspace() : saved_workspace;
  workspace_edit_->setText(ws);
  const int idx = ros_distro_combo_->findData(saved_distro.toLower());
  ros_distro_combo_->setCurrentIndex(idx >= 0 ? idx : 0);

  connect(browse, &QPushButton::clicked, this, &StartupDialog::on_browse_clicked);
  connect(open_studio_button_, &QPushButton::clicked, this, &StartupDialog::on_open_studio_clicked);
  connect(exit_button, &QPushButton::clicked, this, &QDialog::reject);

  update_status("Choose a valid workspace to continue.", false);
}

QString StartupDialog::selected_workspace() const { return workspace_edit_->text().trimmed(); }
QString StartupDialog::selected_ros_distro() const { return ros_distro_combo_->currentData().toString(); }

void StartupDialog::on_browse_clicked()
{
  const QString selected = QFileDialog::getExistingDirectory(this, "Select workspace folder", workspace_edit_->text());
  if (!selected.isEmpty()) {
    workspace_edit_->setText(selected);
  }
}

void StartupDialog::on_open_studio_clicked()
{
  const QString path = selected_workspace();
  if (!workcell_builder::is_valid_workcell_workspace(path)) {
    update_status("Selected folder is not a valid Workcell Studio workspace. Expected a ROS workspace containing src/ and easy_manipulation_deployment, scenes, or assets.", true);
    return;
  }

  QSettings settings("easy_manipulation_deployment", "workcell_builder");
  settings.setValue("startup/last_workspace", path);
  settings.setValue("startup/last_ros_distro", selected_ros_distro());
  accept();
}

void StartupDialog::update_status(const QString & message, bool error)
{
  status_label_->setText(QString("<font color='%1'>%2</font>").arg(error ? "#C0392B" : "#2E86C1", message));
}

QString StartupDialog::default_workspace() const
{
  const QString ws = QDir::homePath() + "/workcell_ws";
  return QDir(ws).exists() ? ws : ws;
}
