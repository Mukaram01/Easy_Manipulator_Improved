#include "external_asset_import_dialog.hpp"
#include "external_asset_importer.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

ExternalAssetImportDialog::ExternalAssetImportDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle("External Asset Import Wizard");

  auto *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(new QLabel("Import External Asset", this));

  auto *form = new QFormLayout();
  source_path_ = new QLineEdit(this);
  auto *browse = new QPushButton("Browse/select external .stl / .urdf / .xacro", this);
  auto *browse_layout = new QHBoxLayout();
  browse_layout->addWidget(source_path_);
  browse_layout->addWidget(browse);
  form->addRow("Select STL / URDF", browse_layout);

  asset_name_ = new QLineEdit(this); form->addRow("Asset Name", asset_name_);
  label_ = new QLineEdit(this); form->addRow("Label", label_);
  category_ = new QLineEdit("Custom / Imported", this); form->addRow("Category", category_);
  asset_type_ = new QComboBox(this); asset_type_->addItems({"fixture", "table", "bin", "custom"}); form->addRow("Asset Type", asset_type_);

  auto mk=[this](){ auto *s=new QDoubleSpinBox(this); s->setRange(-9999.0,9999.0); s->setDecimals(6); return s; };
  dim_x_=mk(); dim_y_=mk(); dim_z_=mk(); form->addRow("Default dimensions x/y/z", dim_x_);
  pose_x_=mk(); pose_y_=mk(); pose_z_=mk(); form->addRow("Default pose x/y/z", pose_x_);
  roll_=mk(); pitch_=mk(); yaw_=mk(); form->addRow("Default pose roll/pitch/yaw", roll_);
  default_z_hint_=mk(); form->addRow("Default Z hint", default_z_hint_);

  license_ = new QLineEdit(this); form->addRow("License", license_);
  source_note_ = new QTextEdit(this); form->addRow("Source Note", source_note_);
  tags_ = new QLineEdit(this); form->addRow("Tags", tags_);
  main_layout->addLayout(form);

  auto *row = new QHBoxLayout();
  validate_button_ = new QPushButton("Validate Imported Asset", this);
  add_to_library_button_ = new QPushButton("Add to Asset Library", this);
  import_and_place_button_ = new QPushButton("Import and Place", this);
  row->addWidget(validate_button_);
  row->addWidget(add_to_library_button_);
  row->addWidget(import_and_place_button_);
  main_layout->addLayout(row);

  connect(browse, &QPushButton::clicked, this, &ExternalAssetImportDialog::onBrowse);
  connect(validate_button_, &QPushButton::clicked, this, &ExternalAssetImportDialog::onValidate);
  connect(add_to_library_button_, &QPushButton::clicked, this, &ExternalAssetImportDialog::onAddToLibrary);
  connect(import_and_place_button_, &QPushButton::clicked, this, &ExternalAssetImportDialog::onImportAndPlace);
}
void ExternalAssetImportDialog::onBrowse() {
  const QString path = QFileDialog::getOpenFileName(this, "Select STL / URDF", QString(), "Assets (*.stl *.urdf *.xacro)");
  if (!path.isEmpty()) source_path_->setText(path);
}
void ExternalAssetImportDialog::onValidate() { QMessageBox::information(this, "Import Summary", "Validate Imported Asset"); }
void ExternalAssetImportDialog::onAddToLibrary() { QMessageBox::information(this, "Import Summary", "Add to Asset Library"); }
void ExternalAssetImportDialog::onImportAndPlace() { QMessageBox::information(this, "Import Summary", "Import and Place"); }
