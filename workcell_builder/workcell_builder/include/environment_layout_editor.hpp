#pragma once

#include <QDialog>
#include <QDoubleSpinBox>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <unordered_map>

#include "object_placement_model.hpp"

namespace workcell_builder
{

class EnvironmentLayoutEditor : public QDialog
{
  Q_OBJECT

public:
  explicit EnvironmentLayoutEditor(QWidget * parent = nullptr);
  void set_objects(const std::vector<PlacedObject> & objects);
  std::vector<PlacedObject> objects() const;

  static double world_metres_to_canvas_pixels(double metres);
  static double canvas_pixels_to_world_metres(double pixels);

private:
  void rebuild_scene();
  void refresh_table();
  void apply_table_pose_to_model();

  ObjectPlacementModel model_;
  QGraphicsView * view_{nullptr};
  QGraphicsScene * scene_{nullptr};
  QTableWidget * table_{nullptr};
  QDoubleSpinBox * grid_size_{nullptr};
  QLabel * mode_label_{nullptr};
  std::unordered_map<QGraphicsRectItem *, std::string> item_to_name_;
};

}  // namespace workcell_builder
