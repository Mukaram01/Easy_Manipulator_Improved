#include "include/asset_picker_dialog.hpp"
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

AssetPickerDialog::AssetPickerDialog(const QString & title, QWidget * parent) : QDialog(parent) {
  setWindowTitle(title);
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("READY assets are selectable. Incomplete assets are shown for manual fallback.", this));
  table_ = new QTableWidget(this);
  table_->setColumnCount(6);
  table_->setHorizontalHeaderLabels({"Label", "Description Package", "MoveIt Config", "URDF/Xacro or Mesh", "Type", "Status"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  layout->addWidget(table_);
  auto * row = new QHBoxLayout();
  auto * refresh = new QPushButton("Refresh", this);
  select_button_ = new QPushButton("Select", this);
  auto * cancel = new QPushButton("Cancel", this);
  row->addWidget(refresh);row->addWidget(select_button_);row->addWidget(cancel);
  layout->addLayout(row);
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
  connect(select_button_, &QPushButton::clicked, this, &AssetPickerDialog::on_select_clicked);
  connect(refresh, &QPushButton::clicked, this, &QDialog::reject);
}
void AssetPickerDialog::set_candidates(const std::vector<AssetCandidate> & candidates, const std::vector<std::string> & searched_paths) {
  table_->setRowCount(static_cast<int>(candidates.size()));
  for (int i=0;i<(int)candidates.size();++i){const auto & c=candidates[i];
    table_->setItem(i,0,new QTableWidgetItem(QString::fromStdString(c.label)));
    table_->setItem(i,1,new QTableWidgetItem(QString::fromStdString(c.description_package)));
    table_->setItem(i,2,new QTableWidgetItem(QString::fromStdString(c.moveit_config_package)));
    table_->setItem(i,3,new QTableWidgetItem(QString::fromStdString(c.urdf_or_xacro)));
    table_->setItem(i,4,new QTableWidgetItem(QString::fromStdString(c.inferred_type)));
    table_->setItem(i,5,new QTableWidgetItem(QString::fromStdString(c.status)));
  }
  if (candidates.empty()) {
    QString details("No assets found. Discovery paths searched:\n");
    for (const auto & p : searched_paths) details += QString::fromStdString(" - "+p+"\n");
    QMessageBox::information(this, "No assets found", details);
  }
}
AssetCandidate AssetPickerDialog::selected_candidate() const { return selected_; }
void AssetPickerDialog::on_select_clicked(){
  const int row = table_->currentRow();
  if (row < 0) { QMessageBox::warning(this, "Selection required", "Please select an asset row."); return; }
  if (table_->item(row,5)->text() != "READY") { QMessageBox::warning(this, "Selected asset is incomplete", "Selected asset is incomplete: missing description and/or moveit config."); return; }
  selected_.label = table_->item(row,0)->text().toStdString();
  selected_.description_package = table_->item(row,1)->text().toStdString();
  selected_.moveit_config_package = table_->item(row,2)->text().toStdString();
  selected_.urdf_or_xacro = table_->item(row,3)->text().toStdString();
  selected_.inferred_type = table_->item(row,4)->text().toStdString();
  selected_.status = table_->item(row,5)->text().toStdString();
  accept();
}
