#include "workcell_builder_ui_utils.hpp"
#include "object_placement_dialog.hpp"

#include <QDialogButtonBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <array>
#include "environment_layout_editor.hpp"

namespace workcell_builder
{

ObjectPlacementDialog::ObjectPlacementDialog(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("Object Placement Manager");
  auto * outer = new QVBoxLayout(this);
  table_ = new QTableWidget(this);
  table_->setColumnCount(10);
  table_->setHorizontalHeaderLabels(
    {"Name", "Source", "Mesh path", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Status / Warnings"});
  table_->horizontalHeader()->setStretchLastSection(true);
  outer->addWidget(table_);

  auto * row = new QHBoxLayout();
  auto mk = [this, row](const QString & t, auto fn) {
      auto * b = new QPushButton(t, this);
      row->addWidget(b);
      QObject::connect(b, &QPushButton::clicked, this, fn);
    };
  mk("Add Asset Object", [this]() { add_default_object("asset_stl"); });
  mk("Import STL to Asset Library", [this]() { add_default_object("imported_stl"); });
  mk("Add Generated Primitive", [this]() { add_default_object("generated_primitive"); });
  mk("Edit Pose", [this]() { QMessageBox::information(this, "Edit Pose", "Edit Pose updates selected row values directly."); });
  mk("Duplicate Object", [this]() {
    const int row_index = table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(model_.objects().size())) return;
    model_.duplicate_object(model_.objects().at(static_cast<size_t>(row_index)).name);
    rebuild_table();
  });
  mk("Remove Object", [this]() {
    const int row_index = table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(model_.objects().size())) return;
    model_.remove_object(model_.objects().at(static_cast<size_t>(row_index)).name);
    rebuild_table();
  });
  mk("Refresh Preview", [this]() { rebuild_table(); });
  mk("Open Visual Layout Editor", [this]() {
    EnvironmentLayoutEditor editor(this);
    editor.setWindowTitle("Open Visual Layout Editor");
    editor.set_objects(model_.objects());
    if (editor.exec() == QDialog::Accepted) {
      model_ = ObjectPlacementModel();
      for (const auto & o : editor.objects()) model_.add_object(o);
      rebuild_table();
    }
  });
  outer->addLayout(row);

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close | QDialogButtonBox::Apply, this);
  QObject::connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  QObject::connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  outer->addWidget(buttons);

  applyCompactDialogDefaults(this);
}

void ObjectPlacementDialog::set_objects(const std::vector<PlacedObject> & objects)
{
  for (const auto & obj : objects) {
    model_.add_object(obj);
  }
  rebuild_table();
}

std::vector<PlacedObject> ObjectPlacementDialog::objects() const { return model_.objects(); }

void ObjectPlacementDialog::rebuild_table()
{
  const auto objects = model_.objects();
  table_->setRowCount(static_cast<int>(objects.size()));
  for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
    const auto & o = objects[static_cast<size_t>(i)];
    const std::array<QString, 10> vals = {
      QString::fromStdString(o.name), QString::fromStdString(o.source_type), QString::fromStdString(o.mesh_path),
      QString::number(o.x), QString::number(o.y), QString::number(o.z), QString::number(o.roll),
      QString::number(o.pitch), QString::number(o.yaw), QString::fromStdString(o.status)
    };
    for (int c = 0; c < 10; ++c) table_->setItem(i, c, new QTableWidgetItem(vals[static_cast<size_t>(c)]));
  }
}

void ObjectPlacementDialog::add_default_object(const std::string & source_type)
{
  bool ok = false;
  const QString entered = QInputDialog::getText(this, "Placed object", "Name", QLineEdit::Normal, "object", &ok);
  if (!ok || entered.isEmpty()) return;
  PlacedObject obj;
  obj.name = sanitize_object_name(entered.toStdString());
  obj.source_type = source_type;
  obj.mesh_path = source_type == "generated_primitive" ? "meshes/generated_objects/object.stl" : "";
  std::string warning;
  if (!validate_placed_object(obj, &warning)) {
    obj.status = warning;
  }
  model_.add_object(obj);
  rebuild_table();
}

}  // namespace workcell_builder
