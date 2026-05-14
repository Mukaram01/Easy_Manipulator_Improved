#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__GUI__STARTUP_DIALOG_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__GUI__STARTUP_DIALOG_H_

#include <QDialog>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;

class StartupDialog : public QDialog
{
  Q_OBJECT

public:
  explicit StartupDialog(QWidget * parent = nullptr);
  QString selected_workspace() const;
  QString selected_ros_distro() const;

private slots:
  void on_browse_clicked();
  void on_open_studio_clicked();

private:
  void update_status(const QString & message, bool error);
  QString default_workspace() const;

  QComboBox * ros_distro_combo_;
  QLineEdit * workspace_edit_;
  QLabel * status_label_;
  QPushButton * open_studio_button_;
};

#endif
