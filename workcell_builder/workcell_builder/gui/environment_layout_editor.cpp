#include "environment_layout_editor.hpp"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGraphicsSimpleTextItem>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace workcell_builder
{

namespace
{
constexpr double kDefaultPixelsPerMetre = 100.0;
}

EnvironmentLayoutEditor::EnvironmentLayoutEditor(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("Open Visual Layout Editor");
  auto * outer = new QVBoxLayout(this);

  auto * toolbar = new QHBoxLayout();
  toolbar->addWidget(new QPushButton("Top-down Layout", this));
  toolbar->addWidget(new QPushButton("Add Object From Placement Manager", this));
  toolbar->addWidget(new QPushButton("Select Object", this));
  toolbar->addWidget(new QPushButton("Move Object", this));
  toolbar->addWidget(new QPushButton("Edit Pose", this));
  toolbar->addWidget(new QPushButton("Snap to Grid", this));
  toolbar->addWidget(new QLabel("Grid Size", this));
  grid_size_ = new QDoubleSpinBox(this);
  grid_size_->setValue(0.1);
  toolbar->addWidget(grid_size_);
  toolbar->addWidget(new QPushButton("Save Layout to Environment YAML", this));
  toolbar->addWidget(new QPushButton("Reload From Environment YAML", this));
  toolbar->addWidget(new QPushButton("Refresh Preview", this));
  outer->addLayout(toolbar);

  mode_label_ = new QLabel("Top-down Layout (QGraphicsView/QGraphicsScene)", this);
  outer->addWidget(mode_label_);

  scene_ = new QGraphicsScene(this);
  view_ = new QGraphicsView(scene_, this);
  outer->addWidget(view_);

  table_ = new QTableWidget(this);
  table_->setColumnCount(10);
  table_->setHorizontalHeaderLabels({"Name", "Source", "Mesh", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Status"});
  table_->horizontalHeader()->setStretchLastSection(true);
  outer->addWidget(table_);

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Apply | QDialogButtonBox::Close, this);
  connect(buttons, &QDialogButtonBox::accepted, this, [this]() { apply_table_pose_to_model(); rebuild_scene(); });
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  outer->addWidget(buttons);
}

double EnvironmentLayoutEditor::world_metres_to_canvas_pixels(double metres) { return metres * kDefaultPixelsPerMetre; }
double EnvironmentLayoutEditor::canvas_pixels_to_world_metres(double pixels) { return pixels / kDefaultPixelsPerMetre; }

void EnvironmentLayoutEditor::set_objects(const std::vector<PlacedObject> & objects)
{
  for (const auto & obj : objects) model_.add_object(obj);
  refresh_table();
  rebuild_scene();
}

std::vector<PlacedObject> EnvironmentLayoutEditor::objects() const { return model_.objects(); }

void EnvironmentLayoutEditor::refresh_table()
{
  const auto objects = model_.objects();
  table_->setRowCount(static_cast<int>(objects.size()));
  for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
    const auto & o = objects[static_cast<size_t>(i)];
    const QString warn = o.mesh_path.empty() ? "missing mesh warning" : "";
    table_->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(o.name)));
    table_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(o.source_type)));
    table_->setItem(i, 2, new QTableWidgetItem(QString::fromStdString(o.mesh_path)));
    table_->setItem(i, 3, new QTableWidgetItem(QString::number(o.x)));
    table_->setItem(i, 4, new QTableWidgetItem(QString::number(o.y)));
    table_->setItem(i, 5, new QTableWidgetItem(QString::number(o.z)));
    table_->setItem(i, 6, new QTableWidgetItem(QString::number(o.roll)));
    table_->setItem(i, 7, new QTableWidgetItem(QString::number(o.pitch)));
    table_->setItem(i, 8, new QTableWidgetItem(QString::number(o.yaw)));
    table_->setItem(i, 9, new QTableWidgetItem(QString::fromStdString(o.status) + (warn.isEmpty() ? "" : " | " + warn)));
  }
}

void EnvironmentLayoutEditor::rebuild_scene()
{
  scene_->clear();
  for (int g = -10; g <= 10; ++g) {
    scene_->addLine(-500, g * 50, 500, g * 50);
    scene_->addLine(g * 50, -500, g * 50, 500);
  }
  scene_->addText("origin (0,0)")->setPos(0, 0);

  for (const auto & o : model_.objects()) {
    auto * rect = scene_->addRect(world_metres_to_canvas_pixels(o.x), world_metres_to_canvas_pixels(o.y), 60, 40);
    rect->setFlag(QGraphicsItem::ItemIsMovable, true);
    rect->setFlag(QGraphicsItem::ItemIsSelectable, true);
    scene_->addText(QString::fromStdString(o.name + " [" + o.source_type + "]"))->setPos(rect->rect().x(), rect->rect().y());
  }
}

void EnvironmentLayoutEditor::apply_table_pose_to_model()
{
  ObjectPlacementModel updated;
  for (int i = 0; i < table_->rowCount(); ++i) {
    PlacedObject o;
    o.name = table_->item(i, 0)->text().toStdString();
    o.source_type = table_->item(i, 1)->text().toStdString();
    o.mesh_path = table_->item(i, 2)->text().toStdString();
    o.x = table_->item(i, 3)->text().toDouble();
    o.y = table_->item(i, 4)->text().toDouble();
    o.z = table_->item(i, 5)->text().toDouble();
    o.roll = table_->item(i, 6)->text().toDouble();
    o.pitch = table_->item(i, 7)->text().toDouble();
    o.yaw = table_->item(i, 8)->text().toDouble();
    o.status = table_->item(i, 9)->text().toStdString();
    updated.add_object(o);
  }
  model_ = updated;
}

}  // namespace workcell_builder
