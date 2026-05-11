#pragma once

#include <QDialog>

class QLineEdit;
class QDoubleSpinBox;
class QPushButton;
class QTextEdit;
class QComboBox;

class ExternalAssetImportDialog : public QDialog {
  Q_OBJECT
public:
  explicit ExternalAssetImportDialog(QWidget *parent = nullptr);

private slots:
  void onBrowse();
  void onValidate();
  void onAddToLibrary();
  void onImportAndPlace();

private:
  QLineEdit *source_path_;
  QLineEdit *asset_name_;
  QLineEdit *label_;
  QLineEdit *category_;
  QComboBox *asset_type_;
  QDoubleSpinBox *dim_x_;
  QDoubleSpinBox *dim_y_;
  QDoubleSpinBox *dim_z_;
  QDoubleSpinBox *pose_x_;
  QDoubleSpinBox *pose_y_;
  QDoubleSpinBox *pose_z_;
  QDoubleSpinBox *roll_;
  QDoubleSpinBox *pitch_;
  QDoubleSpinBox *yaw_;
  QDoubleSpinBox *default_z_hint_;
  QLineEdit *license_;
  QTextEdit *source_note_;
  QLineEdit *tags_;
  QPushButton *validate_button_;
  QPushButton *add_to_library_button_;
  QPushButton *import_and_place_button_;
};
